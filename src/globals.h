#include <Arduino.h>
#include <ArduinoBLE.h>
#include <ResponsiveAnalogRead.h> // smoothing for throttle
#include <ESP32Servo.h> 
#include <Thread.h> // run tasks at different intervals
#include <StaticThreadController.h>
#include <CircularBuffer.h>
#include <ezButton.h>

#define BUTTON_PIN D0
#define THROTTLE_PIN A3
#define ESC_PWM_PIN A2
#define ESC_RX_PIN D7
#define ESC_TX_PIN D6

#define SerialESC Serial1
#define ESC_BAUD_RATE 115200
#define ESC_DATA_SIZE 22
#define ESC_TIMEOUT 10
#define ESC_DISARMED_PWM 1010
#define ESC_MIN_PWM 1030  // ESC min is 1050
#define ESC_MAX_PWM 1990  // ESC max 1950
#define ANALOG_READ_MAX 4090
#define POT_READ_MAX 1400
#define POT_MIN_OFFSET 0  // Pot value delta to actually accelerate to make it less touchy
#define POT_OUT_OF_BOUNDS_VALUE 200 // Stop throttle if values are out of throttle limits (for safety)
#define BATT_MIN_V 49.0  // 7S min (use 42v for 6S)
#define BATT_MAX_V 58.8  // 7S max (use 50v for 6S)
#define INITIALIZED_THRESHOLD 10 // set initial value after pot value is stable (delta < 10)

#define BLE_CONNECTION_THREASHOLD 10000
#define BLE_CONNECTED_THREAD_INTERVAL 1000
#define BLE_CONNECTING_THREAD_INTERVAL 50

// Telemetry
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

class EscTelemetry {
  public:
  float volts;
  float temperatureC;
  float amps;
  float eRPM;
  float inPWM;
  float outPWM;
  uint8_t statusFlag;
  word checksum;
};

// v2 ESC telemetry
typedef struct  {
  // Voltage
  int V_HI;
  int V_LO;

  // Temperature
  int T_HI;
  int T_LO;

  // Current
  int I_HI;
  int I_LO;

  // Reserved
  int R0_HI;
  int R0_LO;

  // eRPM
  int RPM0;
  int RPM1;
  int RPM2;
  int RPM3;

  // Input Duty
  int DUTYIN_HI;
  int DUTYIN_LO;

  // Motor Duty
  int MOTORDUTY_HI;
  int MOTORDUTY_LO;

  // Reserved
  int R1;

  // Status Flags
  int statusFlag;

  // checksum
  int CSUM_HI;
  int CSUM_LO;
} telem_t;

static telem_t raw_telemdata;
