#include <Arduino.h>
#include <ArduinoBLE.h>
#include <ResponsiveAnalogRead.h>
#include <Thread.h>
#include <StaticThreadController.h>
#include <ezButton.h>
#include <WiFi.h>
#include <Adafruit_SleepyDog.h>
#include "globals.h"

void setup() {
  Serial.begin(115200);
  Watchdog.enable(4000);

  // setup power switch button
  powerSwitch.setDebounceTime(50);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // setup Throttle
  analogReadResolution(12);
  pot.setAnalogResolution(4096);
  throttleThread.onRun(handleThrottle);
  throttleThread.setInterval(22);

  // setup ESC
  SerialESC.begin(ESC_BAUD_RATE, SERIAL_8N1, ESC_TX_PIN, ESC_RX_PIN);
  SerialESC.setTimeout(ESC_TIMEOUT);

  esc.attach(ESC_PWM_PIN);
  esc.writeMicroseconds(ESC_DISARMED_PWM);

  trackPowerThread.onRun(handlePowerTracking);
  trackPowerThread.setInterval(250);
  telemetryThread.onRun(handleTelemetry);
  telemetryThread.setInterval(50);

  // setup BLE
  if (!BLE.begin()) {
    Serial.println("Starting Bluetooth@ Low Energy module failed!");
    while(1);
  }
  BLE.setLocalName("SP-140");
  BLE.setAdvertisedService(bleService);
  bleService.addCharacteristic(batteryCharacteristic);
  bleService.addCharacteristic(temperatureCharacteristic);
  bleService.addCharacteristic(voltageCharacteristic);
  bleService.addCharacteristic(ampsCharacteristic);
  bleService.addCharacteristic(rpmCharacteristic);
  bleService.addCharacteristic(kWCharacteristic);
  bleService.addCharacteristic(usedKwhCharacteristic);
  bleService.addCharacteristic(powerCharacteristic);
  BLE.addService(bleService);
  BLE.advertise();

  bleThread.onRun(handleBleData);
  bleThread.setInterval(bleThreadInterval);
}

void loop() {
  threads.run();
}

void handleThrottle() {
  Watchdog.reset();

  pot.update();
  bool containsBigDeltaValues = false;
  int potRaw = ANALOG_READ_MAX - pot.getValue();
  int potLvl = 0;
  for (auto i = 0; i < potBuffer.size(); i++) {
    potLvl += potBuffer[i] / potBuffer.size();
  }
  for (auto i = 0; i < potBuffer.size(); i++) {
    if (abs(potBuffer[i] - potLvl) > POT_READ_MAX * 0.02) {
      containsBigDeltaValues = true;
      break;
    }
  }
  // overwrite values with big deltas but ignore pot changes of less than 2% to reduce noise
  if (containsBigDeltaValues || abs(potRaw - potLvl) > POT_READ_MAX * 0.02) {
    potBuffer.push(potRaw);
  }

  if (millis() - startTime < 2000) return;

  if (prevPotLvl != potLvl) {
    Serial.print("pwmSignal: ");
    Serial.print(pwmSignal);
    Serial.print(" min max: ");
    Serial.print(initialPotLvl);
    Serial.print(" ");
    Serial.print(initialPotLvl + POT_READ_MAX);
    Serial.print(" potLvl: ");
    Serial.print(potLvl);
    Serial.print(" rawPotLvl: ");
    Serial.println(potRaw);
  }

  // set initial potentiometer Lvl if changes are less than INITIALIZED_THRESHOLD
  auto isInitialized = isArmed() && (initialPotLvl != -1 || abs(potLvl - prevPotLvl) < INITIALIZED_THRESHOLD);
  potLvl = limitedThrottle(potLvl, prevPotLvl, 120);

  if (isInitialized && initialPotLvl == -1) {
    initialPotLvl = potLvl;
    Serial.print("initialized to potLvl: ");
    Serial.println(potLvl);
  }

  if (isArmed() && isInitialized) {
    // calculate pwm signal relative to the initial potentiometer Lvl
    pwmSignal = mapd(potLvl - POT_MIN_OFFSET, initialPotLvl, initialPotLvl + POT_READ_MAX, ESC_MIN_PWM, ESC_MAX_PWM);
    
    // set pwm to min if potLvl is out of bounds
    if (constrain(potLvl, initialPotLvl - POT_OUT_OF_BOUNDS_VALUE, initialPotLvl + POT_READ_MAX + POT_OUT_OF_BOUNDS_VALUE) == potLvl) {
      esc.writeMicroseconds(pwmSignal);
    } else {
      esc.writeMicroseconds(ESC_MIN_PWM);
    }
  } else {
    if (!isArmed()) initialPotLvl = -1;
    esc.writeMicroseconds(ESC_DISARMED_PWM);
  }
}

void handlePowerTracking() {
  static unsigned long prevPwrMillis = 0;
  unsigned long currentPwrMillis = millis();
  unsigned long msec_diff = (currentPwrMillis - prevPwrMillis);  // eg 0.30 sec
  prevPwrMillis = currentPwrMillis;

  if (isArmed()) {
    wattsHoursUsed += round(watts/60/60*msec_diff)/1000.0;
  }
}

void handleTelemetry() {
  while (SerialESC.available() > 0) {
    SerialESC.read();
  }
  SerialESC.readBytes(escData, ESC_DATA_SIZE);
  if (enforceFletcher16()) {
    parseTelemetryData();
  }
}

void handleBleData() {
  // change thread interval if BLE is connected vs connecting so connection process is faster
  auto isBleConnectionThresholdReached = millis() - bleConnectedTime > BLE_CONNECTION_THREASHOLD;
  if (!isBleConnectionThresholdReached && bleThreadInterval != BLE_CONNECTING_THREAD_INTERVAL) {
    bleThread.setInterval(bleThreadInterval = BLE_CONNECTING_THREAD_INTERVAL);
  } else if (isBleConnectionThresholdReached && bleThreadInterval != BLE_CONNECTED_THREAD_INTERVAL) {
    bleThread.setInterval(bleThreadInterval = BLE_CONNECTED_THREAD_INTERVAL);
  }

  if (!central || !central.connected()) {
    bleConnectedTime = millis();
    central = BLE.central();
    return;
  }

  bleData.volts = smoothedBatteryVoltage();
  bleData.batteryPercentage = batteryPercentage(bleData.volts);
  bleData.amps = telemetryData.amps;
  bleData.kW = constrain(watts / 1000.0, 0, 50);
  bleData.usedKwh = wattsHoursUsed / 1000;
  bleData.power = mapd(pwmSignal, ESC_MIN_PWM, ESC_MAX_PWM, 0, 100);
  //TODO: rpm, temperature

  batteryCharacteristic.writeValue(bleData.batteryPercentage);
  voltageCharacteristic.writeValue(bleData.volts);
  temperatureCharacteristic.writeValue(bleData.temperatureC);
  rpmCharacteristic.writeValue(bleData.rpm);
  ampsCharacteristic.writeValue(bleData.amps);
  usedKwhCharacteristic.writeValue(bleData.usedKwh);
  kWCharacteristic.writeValue(bleData.kW);
  powerCharacteristic.writeValue(bleData.power);
}

// /// region Helper functions

// throttle easing function based on threshold/performance mode
int limitedThrottle(int current, int last, int threshold) {
  if (current - last >= threshold) {  // accelerating too fast. limit
    int limitedThrottle = last + threshold;
    prevPotLvl = limitedThrottle;  // save for next time
    return limitedThrottle;
  } else if (last - current >= threshold * 2) {  // decelerating too fast. limit
    int limitedThrottle = last - threshold * 2;  // double the decel vs accel
    prevPotLvl = limitedThrottle;  // save for next time
    return limitedThrottle;
  }
  prevPotLvl = current;
  return current;
}

bool isArmed() {
  powerSwitch.loop();
  return powerSwitch.getState() == LOW;
  // return analogRead(BUTTON_PIN) == LOW;
}

double mapd(double x, double in_min, double in_max, double out_min, double out_max) {
  return (constrain(x, in_min, in_max) - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void parseTelemetryData() {
  // LSB First
  // TODO is this being called even with no ESC?

  uint16_t _volts = word(escData[1], escData[0]);
  telemetryData.volts = _volts / 100.0;

  if (telemetryData.volts > BATT_MIN_V) {
    telemetryData.volts += 1.5;  // calibration
  }

  if (telemetryData.volts > 1) {  // ignore empty data
    voltageBuffer.push(telemetryData.volts);
  }

  uint16_t _temperatureC = word(escData[3], escData[2]);
  telemetryData.temperatureC = _temperatureC/100.0;
  // reading 17.4C = 63.32F in 84F ambient?

  int16_t _amps = word(escData[5], escData[4]);
  telemetryData.amps = _amps;

  watts = telemetryData.amps * telemetryData.volts;

  // 7 and 6 are reserved bytes
  int16_t _eRPM = escData[11];     // 0
  _eRPM << 8;
  _eRPM += escData[10];    // 0
  _eRPM << 8;
  _eRPM += escData[9];     // 30
  _eRPM << 8;
  _eRPM += escData[8];     // b4
  telemetryData.eRPM = _eRPM/6.0/2.0;

  uint16_t _inPWM = word(escData[13], escData[12]);
  telemetryData.inPWM = _inPWM/100.0;

  uint16_t _outPWM = word(escData[15], escData[14]);
  telemetryData.outPWM = _outPWM/100.0;

  // 17 and 16 are reserved bytes
  // 19 and 18 is checksum
  telemetryData.checksum = word(escData[19], escData[18]);
}

// run checksum and return true if valid
bool enforceFletcher16() {
  static unsigned long transmitted = 0;
  static unsigned long failed = 0;

  // Check checksum, revert to previous data if bad:
  word checksum = (unsigned short)(word(escData[19], escData[18]));
  unsigned char sum1 = 0;
  unsigned char sum2 = 0;
  unsigned short sum = 0;
  for (int i = 0; i < ESC_DATA_SIZE-2; i++) {
    sum1 = (unsigned char)(sum1 + escData[i]);
    sum2 = (unsigned char)(sum2 + sum1);
  }
  sum = (unsigned char)(sum1 - sum2);
  sum = sum << 8;
  sum |= (unsigned char)(sum2 - 2*sum1);

  if (sum != checksum) {
    failed++;
    if (failed >= 1000) { // keep track of how reliable the transmission is
      transmitted = 1;
      failed = 0;
    }
    return false;
  }
  return true;
}

float smoothedBatteryVoltage() {
  float avg = 0.0;

  if (voltageBuffer.isEmpty()) { 
    return avg;
  }
  using index_t = decltype(voltageBuffer)::index_t;
  for (index_t i = 0; i < voltageBuffer.size(); i++) {
    avg += voltageBuffer[i] / voltageBuffer.size();
  }
  return avg;
}


// simple set of data points from load testing
// maps voltage to battery percentage
float batteryPercentage(float voltage) {
  float battPercent = 0;

  if (voltage > 94.8) {
    battPercent = mapd(voltage, 94.8, 99.6, 90, 100);
  } else if (voltage > 93.36) {
    battPercent = mapd(voltage, 93.36, 94.8, 80, 90);
  } else if (voltage > 91.68) {
    battPercent = mapd(voltage, 91.68, 93.36, 70, 80);
  } else if (voltage > 89.76) {
    battPercent = mapd(voltage, 89.76, 91.68, 60, 70);
  } else if (voltage > 87.6) {
    battPercent = mapd(voltage, 87.6, 89.76, 50, 60);
  } else if (voltage > 85.2) {
    battPercent = mapd(voltage, 85.2, 87.6, 40, 50);
  } else if (voltage > 82.32) {
    battPercent = mapd(voltage, 82.32, 85.2, 30, 40);
  } else if (voltage > 80.16) {
    battPercent = mapd(voltage, 80.16, 82.32, 20, 30);
  } else if (voltage > 78) {
    battPercent = mapd(voltage, 78, 80.16, 10, 20);
  } else if (voltage > 60.96) {
    battPercent = mapd(voltage, 60.96, 78, 0, 10);
  }
  return constrain(battPercent, 0, 100);
}

// inspired by https://github.com/rlogiacco/BatterySense/
// https://www.desmos.com/calculator/7m9lu26vpy
uint8_t battery_sigmoidal(float voltage, uint16_t minVoltage, uint16_t maxVoltage) {
  uint8_t result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage)/(maxVoltage - minVoltage), 5.5)));
  return result >= 100 ? 100 : result;
}

/// endregion
