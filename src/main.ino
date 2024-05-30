#include <Arduino.h>
#include <ArduinoBLE.h>
#include <ResponsiveAnalogRead.h>
#include <Thread.h>
#include <StaticThreadController.h>
#include <ezButton.h>
#include <WiFi.h>
#include <Adafruit_SleepyDog.h>
#include "globals.h"
#include "CruiseControl.h"
#include "Helper.h"
#include "StringPrinter.h"

// Power Switch
ezButton powerSwitch(BUTTON_PIN);
bool isArmed = false;

// Throttle
ResponsiveAnalogRead pot(THROTTLE_PIN, false);
CircularBuffer<int, 19> potBuffer;
CruiseControl cruiseControl;
auto startTime = millis();
int prevPotLvl = 0;
int initialPotLvl = -1;
int pwmSignal = 0;

// ESC
EscTelemetry telemetryData = EscTelemetry();
Servo esc;
CircularBuffer<float, 50> voltageBuffer;
float wattsHoursUsed = 0;
float watts = 0;
byte escDataV2[ESC_DATA_SIZE];

// Bluetooth® Low Energy LED Service
auto bleConnectedTime = millis();
auto bleThreadInterval = BLE_CONNECTING_THREAD_INTERVAL;
StringPrinter bleSerial;
BleData bleData = BleData();
BLEService bleService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEDevice central;
BLEDoubleCharacteristic batteryCharacteristic("00000000-0019-b100-01e8-f2537e4f6c00", BLERead | BLENotify);
BLEDoubleCharacteristic voltageCharacteristic("00000000-0019-b100-01e8-f2537e4f6c01", BLERead | BLENotify);
BLEDoubleCharacteristic temperatureCharacteristic("00000000-0019-b100-01e8-f2537e4f6c02", BLERead | BLENotify);
BLEDoubleCharacteristic ampsCharacteristic("00000000-0019-b100-01e8-f2537e4f6c03", BLERead | BLENotify);
BLEDoubleCharacteristic rpmCharacteristic("00000000-0019-b100-01e8-f2537e4f6c04", BLERead | BLENotify);
BLEDoubleCharacteristic kWCharacteristic("00000000-0019-b100-01e8-f2537e4f6c05", BLERead | BLENotify);
BLEDoubleCharacteristic usedKwhCharacteristic("00000000-0019-b100-01e8-f2537e4f6c06", BLERead | BLENotify);
BLEDoubleCharacteristic powerCharacteristic("00000000-0019-b100-01e8-f2537e4f6c07", BLERead | BLENotify);
BLEBoolCharacteristic armedCharacteristic("00000000-0019-b100-01e8-f2537e4f6c08", BLERead | BLENotify);
BLEBoolCharacteristic cruiseCharacteristic("00000000-0019-b100-01e8-f2537e4f6c09", BLERead | BLENotify);
BLEDoubleCharacteristic altitudeCharacteristic("00000000-0019-b100-01e8-f2537e4f6c10", BLEWriteWithoutResponse | BLENotify);
BLECharacteristic logCharacteristic("00000000-0020-b100-01e8-f2537e4f6c00", BLERead | BLENotify, 20);

Thread powerSwitchThread = Thread();
Thread throttleThread = Thread();
Thread telemetryThread = Thread();
Thread trackPowerThread = Thread();
Thread bleThread = Thread();
Thread bleLogThread = Thread();
StaticThreadController<6> threads(&powerSwitchThread, &throttleThread, &telemetryThread, &trackPowerThread, &bleThread, &bleLogThread);

void setup() {
  Serial.begin(115200);
  Watchdog.enable(4000);

  // setup power switch button
  powerSwitch.setDebounceTime(50);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  powerSwitchThread.onRun(handlePowerSwitch);
  powerSwitchThread.setInterval(22);

  // setup Throttle
  analogReadResolution(12);
  pot.setAnalogResolution(4096);
  throttleThread.onRun(handleThrottle);
  throttleThread.setInterval(22);

  // setup ESC
  SerialESC.begin(ESC_BAUD_RATE, 134217756UL, ESC_TX_PIN, ESC_RX_PIN);
  SerialESC.setTimeout(ESC_TIMEOUT);

  esc.attach(ESC_PWM_PIN);
  esc.writeMicroseconds(ESC_DISARMED_PWM);

  trackPowerThread.onRun(handlePowerTracking);
  trackPowerThread.setInterval(250);

  telemetryThread.onRun(handleTelemetry);
  telemetryThread.setInterval(50);

  // setup BLE
  if (!BLE.begin()) {
    bleSerial.println("Starting Bluetooth@ Low Energy module failed!");
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
  bleService.addCharacteristic(armedCharacteristic);
  bleService.addCharacteristic(cruiseCharacteristic);
  bleService.addCharacteristic(altitudeCharacteristic);
  bleService.addCharacteristic(logCharacteristic);

  BLE.addService(bleService);
  BLE.advertise();

  bleThread.onRun(handleBleData);
  bleThread.setInterval(bleThreadInterval);
  bleLogThread.onRun(handleBleLog);
  bleLogThread.setInterval(100);
}

void loop() {
  threads.run();
}

void handlePowerSwitch() {
  powerSwitch.loop();
  auto armed = powerSwitch.getState() == LOW;
  static auto lastSwitchOff = millis();
  static bool isSwitchOn = armed;
  bool hasSwitched = armed != isSwitchOn;
  if (hasSwitched) {
    isSwitchOn = armed;
    if (!armed) lastSwitchOff = millis();
  }
  auto isQuickToggled = millis() - lastSwitchOff < 500;

  if (hasSwitched && armed && isQuickToggled && cruiseControl.hasRequiredAltitude()) {
    cruiseControl.enable();
    bleSerial.println("cruise enabled");
  }

  if (armed) {
    if (!isArmed) bleSerial.println("armed");
    isArmed = true;
  } else if (!isQuickToggled) {
    if (isArmed) bleSerial.println("disarmed");
    isArmed = false;
  }
}

void handleThrottle() {
  Watchdog.reset();

  pot.update();
  int potRaw = ANALOG_READ_MAX - pot.getValue();
  int potLvl = 0;
  for (auto i = 0; i < potBuffer.size(); i++) {
    potLvl += potBuffer[i] / potBuffer.size();
  }
  potBuffer.push(potRaw);

  if (millis() - startTime < 2000) return;

  if (abs(prevPotLvl - potLvl) > 10) {
    bleSerial.print("pwmSignal: ");
    bleSerial.print(pwmSignal);
    bleSerial.print(" min max: ");
    bleSerial.print(initialPotLvl);
    bleSerial.print(" ");
    bleSerial.print(initialPotLvl + POT_READ_MAX);
    bleSerial.print(" potLvl: ");
    bleSerial.print(potLvl);
    bleSerial.print(" rawPotLvl: ");
    bleSerial.println(potRaw);
  }

  // set initial potentiometer Lvl once value changes are less than INITIALIZED_THRESHOLD
  auto isInitialized = isArmed && (initialPotLvl != -1 || abs(potLvl - prevPotLvl) < INITIALIZED_THRESHOLD);
  potLvl = limitedThrottle(potLvl, prevPotLvl, 120);

  if (isInitialized && initialPotLvl == -1) {
    initialPotLvl = potLvl;
    bleSerial.print("init to potLvl: ");
    bleSerial.println(potLvl);
  }

  if (isArmed && isInitialized) {
    // calculate pwm signal relative to the initial potentiometer Lvl
    pwmSignal = mapd(potLvl - POT_MIN_OFFSET, initialPotLvl, initialPotLvl + POT_READ_MAX, ESC_MIN_PWM, ESC_MAX_PWM);
    bool isPotWithinBounds = constrain(potLvl, initialPotLvl - POT_OUT_OF_BOUNDS_VALUE, initialPotLvl + POT_READ_MAX + POT_OUT_OF_BOUNDS_VALUE) == potLvl;
    
    if (isPotWithinBounds) {
      if (cruiseControl.isEnabled() && !cruiseControl.isInitialized()) {
        cruiseControl.initialize(pwmSignal);
      }
      if (pwmSignal > cruiseControl.calculateCruisePwm() * 1.05) {
        if (cruiseControl.isEnabled()) bleSerial.println("cruise disabled");
        cruiseControl.disable();
      }
      if (cruiseControl.isInitialized()) {
        esc.writeMicroseconds(cruiseControl.calculateCruisePwm());
      } else {
        esc.writeMicroseconds(pwmSignal);
      }
    } else {
      esc.writeMicroseconds(ESC_MIN_PWM);
    }
  } else {
    if (!isArmed) {
      if (cruiseControl.isEnabled()) bleSerial.println("cruise disabled");
      initialPotLvl = -1;
      cruiseControl.disable();
    }
    esc.writeMicroseconds(ESC_DISARMED_PWM);
  }
}

void handlePowerTracking() {
  static unsigned long prevPwrMillis = 0;
  unsigned long currentPwrMillis = millis();
  unsigned long msec_diff = (currentPwrMillis - prevPwrMillis);  // eg 0.30 sec
  prevPwrMillis = currentPwrMillis;

  if (isArmed) {
    wattsHoursUsed += round(watts/60/60*msec_diff)/1000.0;
  }
}

void handleTelemetry() {
  prepareSerialRead();
  SerialESC.readBytes(escDataV2, ESC_DATA_SIZE);
  handleSerialData(escDataV2);
}

void handleBleLog() {
  auto chunk = bleSerial.readChunk();
  logCharacteristic.writeValue(chunk.c_str(), chunk.length());
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
  bleData.power = mapd(max(pwmSignal, cruiseControl.calculateCruisePwm()), ESC_MIN_PWM, ESC_MAX_PWM, 0, 100);
  bleData.rpm = telemetryData.eRPM;
  bleData.temperatureC = telemetryData.temperatureC;

  // write values
  batteryCharacteristic.writeValue(bleData.batteryPercentage);
  voltageCharacteristic.writeValue(bleData.volts);
  temperatureCharacteristic.writeValue(bleData.temperatureC);
  rpmCharacteristic.writeValue(bleData.rpm);
  ampsCharacteristic.writeValue(bleData.amps);
  usedKwhCharacteristic.writeValue(bleData.usedKwh);
  kWCharacteristic.writeValue(bleData.kW);
  powerCharacteristic.writeValue(bleData.power);
  armedCharacteristic.writeValue(isArmed);
  cruiseCharacteristic.writeValue(cruiseControl.isEnabled());

  // read values
  cruiseControl.setCurrentAltitude(altitudeCharacteristic.value());
}

/// region Helper functions

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

void prepareSerialRead() {  // TODO needed?
  while (SerialESC.available() > 0) {
    SerialESC.read();
  }
}

void handleSerialData(byte buffer[]) {
  if (buffer[20] != 255 || buffer[21] != 255) {
    //bleSerial.println("no stop byte");
    return; //Stop byte of 65535 not received
  }

  //Check the fletcher checksum
  int checkFletch = checkFletcher16(buffer);

  // checksum
  raw_telemdata.CSUM_HI = buffer[19];
  raw_telemdata.CSUM_LO = buffer[18];

  //TODO alert if no new data in 3 seconds
  int checkCalc = (int)(((raw_telemdata.CSUM_HI << 8) + raw_telemdata.CSUM_LO));

  // Checksums do not match
  if (checkFletch != checkCalc) {
    return;
  }
  // Voltage
  raw_telemdata.V_HI = buffer[1];
  raw_telemdata.V_LO = buffer[0];

  float voltage = (raw_telemdata.V_HI << 8 | raw_telemdata.V_LO) / 100.0;
  telemetryData.volts = voltage; //Voltage

  if (telemetryData.volts > BATT_MIN_V) {
    telemetryData.volts += 1.0; // calibration
  }

  voltageBuffer.push(telemetryData.volts);

  // Temperature
  raw_telemdata.T_HI = buffer[3];
  raw_telemdata.T_LO = buffer[2];

  float rawVal = (float)((raw_telemdata.T_HI << 8) + raw_telemdata.T_LO);

  static int SERIESRESISTOR = 10000;
  static int NOMINAL_RESISTANCE = 10000;
  static int NOMINAL_TEMPERATURE = 25;
  static int BCOEFFICIENT = 3455;

  //convert value to resistance
  float Rntc = (4096 / (float) rawVal) - 1;
  Rntc = SERIESRESISTOR / Rntc;

  // Get the temperature
  float temperature = Rntc / (float) NOMINAL_RESISTANCE; // (R/Ro)
  temperature = (float) log(temperature); // ln(R/Ro)
  temperature /= BCOEFFICIENT; // 1/B * ln(R/Ro)

  temperature += 1.0 / ((float) NOMINAL_TEMPERATURE + 273.15); // + (1/To)
  temperature = 1.0 / temperature; // Invert
  temperature -= 273.15; // convert to Celsius

  // filter bad values
  if (temperature < 0 || temperature > 200) {
    telemetryData.temperatureC = __FLT_MIN__;
  } else {
    temperature = (float) trunc(temperature * 100) / 100; // 2 decimal places
    telemetryData.temperatureC = temperature;
  }

  // Current
  auto _amps = word(buffer[5], buffer[4]) / 12.5;
  if (_amps < 300) telemetryData.amps = _amps; // filter bad values

  watts = telemetryData.amps * telemetryData.volts;

  // Reserved
  raw_telemdata.R0_HI = buffer[7];
  raw_telemdata.R0_LO = buffer[6];

  // eRPM
  raw_telemdata.RPM0 = buffer[11];
  raw_telemdata.RPM1 = buffer[10];
  raw_telemdata.RPM2 = buffer[9];
  raw_telemdata.RPM3 = buffer[8];

  int poleCount = 62;
  int currentERPM = (int)((raw_telemdata.RPM0 << 24) + (raw_telemdata.RPM1 << 16) + (raw_telemdata.RPM2 << 8) + (raw_telemdata.RPM3 << 0)); //ERPM output
  int currentRPM = currentERPM / poleCount;  // Real RPM output
  telemetryData.eRPM = currentRPM;

  // Input Duty
  raw_telemdata.DUTYIN_HI = buffer[13];
  raw_telemdata.DUTYIN_LO = buffer[12];

  int throttleDuty = (int)(((raw_telemdata.DUTYIN_HI << 8) + raw_telemdata.DUTYIN_LO) / 10);
  telemetryData.inPWM = (throttleDuty / 10); //Input throttle

  // Motor Duty
  raw_telemdata.MOTORDUTY_HI = buffer[15];
  raw_telemdata.MOTORDUTY_LO = buffer[14];

  int motorDuty = (int)(((raw_telemdata.MOTORDUTY_HI << 8) + raw_telemdata.MOTORDUTY_LO) / 10);
  int currentMotorDuty = (motorDuty / 10); //Motor duty cycle

  // Reserved
  // raw_telemdata.R1 = buffer[17];

  /* Status Flags
  # Bit position in byte indicates flag set, 1 is set, 0 is default
  # Bit 0: Motor Started, set when motor is running as expected
  # Bit 1: Motor Saturation Event, set when saturation detected and power is reduced for desync protection
  # Bit 2: ESC Over temperature event occurring, shut down method as per configuration
  # Bit 3: ESC Overvoltage event occurring, shut down method as per configuration
  # Bit 4: ESC Undervoltage event occurring, shut down method as per configuration
  # Bit 5: Startup error detected, motor stall detected upon trying to start*/
  raw_telemdata.statusFlag = buffer[16];
  telemetryData.statusFlag = raw_telemdata.statusFlag;
  // bleSerial.print("status ");
  // bleSerial.print(raw_telemdata.statusFlag, BIN);
  // bleSerial.print(" - ");
  // bleSerial.println(" ");
}

int checkFletcher16(byte byteBuffer[]) {
    int fCCRC16;
    int i;
    int c0 = 0;
    int c1 = 0;

    // Calculate checksum intermediate bytesUInt16
    for (i = 0; i < 18; i++) //Check only first 18 bytes, skip crc bytes
    {
        c0 = (int)(c0 + ((int)byteBuffer[i])) % 255;
        c1 = (int)(c1 + c0) % 255;
    }
    // Assemble the 16-bit checksum value
    fCCRC16 = ( c1 << 8 ) | c0;
    return (int)fCCRC16;
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
