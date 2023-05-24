#include <Arduino.h>
#include <ArduinoBLE.h>
#include <ResponsiveAnalogRead.h> // smoothing for throttle
#include <ESP32Servo.h> 
#include <Thread.h> // run tasks at different intervals
#include <StaticThreadController.h>
#include <CircularBuffer.h>

#define BUTTON_PIN D1
#define THROTTLE_PIN A3
#define ESC_PWM_PIN A2
#define ESC_RX_PIN D7
#define ESC_TX_PIN D6

#define SerialESC Serial1
#define ESC_BAUD_RATE 115200
#define ESC_DATA_SIZE 20
#define ESC_TIMEOUT 10
#define ESC_DISARMED_PWM 1010
#define ESC_MIN_PWM 1030  // ESC min is 1050
#define ESC_MAX_PWM 1990  // ESC max 1950
#define ANALOG_READ_MAX 4090
#define POT_READ_MAX (4090 / 4)
#define BATT_MIN_V 49.0  // 7S min (use 42v for 6S)
#define BATT_MAX_V 58.8  // 7S max (use 50v for 6S)
#define INITIALIZED_THRESHOLD 10 // set initial value after pot value is table (delta < 10)

class EscTelemetry {
  public:
  float volts;
  float temperatureC;
  float amps;
  float eRPM;
  float inPWM;
  float outPWM;
  uint8_t status_flags;
  word checksum;
};

class BleData {
  public:
  float volts;
  float temperatureC;
  float amps;
  float rpm;
  float kW;
  float usedKwh;
  float batteryPercentage;
  float power;
};

// Power Switch
ezButton powerSwitch(D1);
auto startTime = millis();

// ESC
EscTelemetry telemetryData = EscTelemetry();
Servo esc;
CircularBuffer<float, 50> voltageBuffer;
float wattsHoursUsed = 0;
float watts = 0;
byte escData[ESC_DATA_SIZE];

// Throttle
ResponsiveAnalogRead pot(THROTTLE_PIN, false);
CircularBuffer<int, 19> potBuffer;
int prevPotLvl = 0;
int initialPotLvl = -1;
int pwmSignal = 0;

// Bluetooth® Low Energy LED Service
BleData bleData = BleData();
BLEService bleService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEDevice central;
BLEDoubleCharacteristic batteryCharacteristic("00002a19-0000-1000-8000-00805f9b34fb", BLERead | BLENotify);
BLEDoubleCharacteristic voltageCharacteristic("00000000-0019-b100-01e8-f2537e4f6c01", BLERead | BLENotify);
BLEDoubleCharacteristic temperatureCharacteristic("00000000-0019-b100-01e8-f2537e4f6c02", BLERead | BLENotify);
BLEDoubleCharacteristic ampsCharacteristic("00000000-0019-b100-01e8-f2537e4f6c03", BLERead | BLENotify);
BLEDoubleCharacteristic rpmCharacteristic("00000000-0019-b100-01e8-f2537e4f6c04", BLERead | BLENotify);
BLEDoubleCharacteristic kWCharacteristic("00000000-0019-b100-01e8-f2537e4f6c05", BLERead | BLENotify);
BLEDoubleCharacteristic usedKwhCharacteristic("00000000-0019-b100-01e8-f2537e4f6c06", BLERead | BLENotify);
BLEDoubleCharacteristic powerCharacteristic("00000000-0019-b100-01e8-f2537e4f6c07", BLERead | BLENotify);

Thread throttleThread = Thread();
Thread telemetryThread = Thread();
Thread trackPowerThread = Thread();
Thread bleThread = Thread();
StaticThreadController<4> threads(&throttleThread, &telemetryThread, &trackPowerThread, &bleThread);
