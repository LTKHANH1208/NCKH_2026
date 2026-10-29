#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <esp_arduino_version.h>

#include "imu.h"
#include "ekf.h"

// ESP32 classic / ESP32-WROOM-32. Arduino-ESP32 core 2.x or 3.x.
#define GPS_RX      16
#define CAMERA_ADDR 0x42

// Each HW-039 (IBT-2 / BTS7960) controls ONE brushed DC motor.
#define RPWM_1 25
#define LPWM_1 26
#define RPWM_2 23
#define LPWM_2 27  // GPIO24 does not exist on classic ESP32.

constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr uint8_t PWM_RESOLUTION = 8; // 0...255
constexpr int16_t TEST_SPEED = 120;    // ~47% duty, adjust if needed
constexpr int16_t DRIVE_SPEED = 120;   // Straight-line speed (0...255)
constexpr int16_t TURN_SPEED = 120;    // One-wheel pivot-turn speed
// Set either sign to -1 if that wheel spins backward on CMD_FORWARD.
constexpr int8_t LEFT_FORWARD_SIGN = 1;
constexpr int8_t RIGHT_FORWARD_SIGN = 1;

IMU imu;
EKF ekf(0.01);
HardwareSerial gpsPort(2);

char nmea[128];
size_t nmeaLen = 0;
bool originSet = false, gpsFix = false;
double lat0 = 0, lon0 = 0, gpsX = 0, gpsY = 0;
uint32_t lastFixMs = 0, lastImuUs = 0, lastCameraMs = 0, lastPrintMs = 0;

enum MoveCommand : uint8_t {
  CMD_STOP = 0, CMD_LEFT = 1, CMD_FORWARD = 2, CMD_RIGHT = 3
};

struct MotorPins {
  uint8_t rpwm;
  uint8_t lpwm;
  uint8_t enable;
};

const MotorPins motor1 = {RPWM_1, LPWM_1};
const MotorPins motor2 = {RPWM_2, LPWM_2};

// Arduino-ESP32 2.x uses PWM channels; 3.x writes directly by GPIO.
static void writePWM(uint8_t pin, uint8_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  uint8_t channel = 0;
  switch (pin) {
    case RPWM_1: channel = 0; break;
    case LPWM_1: channel = 1; break;
    case RPWM_2: channel = 2; break;
    case LPWM_2: channel = 3; break;
    default: return;
  }
  ledcWrite(channel, duty);
#endif
}

static bool attachPWM(uint8_t pin, uint8_t channel) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)channel;
  return ledcAttach(pin, PWM_FREQUENCY, PWM_RESOLUTION);
#else
  if (ledcSetup(channel, PWM_FREQUENCY, PWM_RESOLUTION) == 0) return false;
  ledcAttachPin(pin, channel);
  return true;
#endif
}

static void stopOneMotor(const MotorPins& m) {
  writePWM(m.rpwm, 0);
  writePWM(m.lpwm, 0);
  digitalWrite(m.enable, LOW); // disable outputs, motor coasts
}

static void stopMotors() {
  stopOneMotor(motor1);
  stopOneMotor(motor2);
}

// speed = -255...255: positive forward, negative reverse, zero stop.
// "Forward" is relative to motor wiring. Swap motor leads if necessary.
static void setMotor(const MotorPins& m, int16_t speed) {
  speed = constrain(speed, -255, 255);

  // First disable the bridge and both PWM lines before changing direction.
  stopOneMotor(m);
  if (speed == 0) return;

  digitalWrite(m.enable, HIGH);
  if (speed > 0) writePWM(m.rpwm, (uint8_t)speed);
  else           writePWM(m.lpwm, (uint8_t)(-speed));
}

static bool beginMotors() {

  // Prevent undefined PWM levels at startup.
  const uint8_t pins[] = {RPWM_1, LPWM_1, RPWM_2, LPWM_2};
  for (uint8_t pin : pins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  for (uint8_t i = 0; i < 4; ++i) {
    if (!attachPWM(pins[i], i)) return false;
    writePWM(pins[i], 0);
  }
  stopMotors();
  return true;
}

struct TestStep {
  uint8_t motor;   // 0 = both stopped; 1 or 2 = motor number
  int16_t speed;
  uint32_t durationMs;
  const char* text;
};

const TestStep testSteps[] = {
  {1,  TEST_SPEED, 2000, "Motor 1 FORWARD"},
  {0,  0,          1000, "STOP"},
  {2,  TEST_SPEED, 2000, "Motor 2 FORWARD"},
  {0,  0,          1000, "STOP"},
  {1, -TEST_SPEED, 2000, "Motor 1 REVERSE"},
  {0,  0,          1000, "STOP"},
  {2, -TEST_SPEED, 2000, "Motor 2 REVERSE"},
  {0,  0,          1000, "STOP"}
};

constexpr size_t TEST_STEP_COUNT = sizeof(testSteps) / sizeof(testSteps[0]);
bool testRunning = false;
size_t currentStep = 0;
uint32_t testStepStartMs = 0;
bool cameraEnabled = true;       // Camera controls motors after initialization
uint8_t lastAppliedCameraCommand = CMD_STOP;

// Motor 1 = LEFT wheel; motor 2 = RIGHT wheel.
// A pivot turn stops the inside wheel and drives the outside wheel.
static void applyCameraCommand(uint8_t command) {
  if (!cameraEnabled || testRunning) return;
  if (command > CMD_RIGHT) command = CMD_STOP;
  if (command == lastAppliedCameraCommand) return;

  // Stop both motors before any change of direction/movement.
  stopMotors();
  switch (command) {
    case CMD_FORWARD:
      setMotor(motor1, LEFT_FORWARD_SIGN * DRIVE_SPEED);
      setMotor(motor2, RIGHT_FORWARD_SIGN * DRIVE_SPEED);
      Serial.println("[CAMERA] FORWARD - both motors forward");
      break;
    case CMD_LEFT:
      setMotor(motor2, RIGHT_FORWARD_SIGN * TURN_SPEED);
      Serial.println("[CAMERA] LEFT - right motor forward");
      break;
    case CMD_RIGHT:
      setMotor(motor1, LEFT_FORWARD_SIGN * TURN_SPEED);
      Serial.println("[CAMERA] RIGHT - left motor forward");
      break;
    case CMD_STOP:
    default:
      Serial.println("[CAMERA] STOP - both motors off");
      break;
  }
  lastAppliedCameraCommand = command;
}

static void applyTestStep(size_t index) {
  stopMotors();
  const TestStep& step = testSteps[index];
  if (step.motor == 1) setMotor(motor1, step.speed);
  if (step.motor == 2) setMotor(motor2, step.speed);
  testStepStartMs = millis();
  Serial.printf("[MOTOR TEST %u/%u] %s, speed=%d\n",
                (unsigned)(index + 1), (unsigned)TEST_STEP_COUNT,
                step.text, step.speed);
}

static void startMotorTest() {
  cameraEnabled = false;  // Test mode has exclusive motor control.
  stopMotors();
  lastAppliedCameraCommand = CMD_STOP;
  currentStep = 0;
  testRunning = true;
  applyTestStep(currentStep);
}

static void abortMotorTest() {
  testRunning = false;
  cameraEnabled = false;  // Latched manual stop; camera cannot restart motors.
  stopMotors();
  lastAppliedCameraCommand = CMD_STOP;
  Serial.println("MOTORS STOPPED");
  Serial.println("Send 'r' to re-enable camera driving.");
}

static void pollSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == 's' || c == 'S') abortMotorTest();
    else if (c == 't' || c == 'T') {
      if (!testRunning) startMotorTest();
    } else if (c == 'r' || c == 'R') {
      testRunning = false;
      stopMotors();
      lastAppliedCameraCommand = CMD_STOP;
      cameraEnabled = true;
      Serial.println("CAMERA DRIVE ENABLED");
    }
  }
}

static void updateMotorTest() {
  if (!testRunning) return;
  if ((uint32_t)(millis() - testStepStartMs) < testSteps[currentStep].durationMs)
    return;

  stopMotors();
  ++currentStep;
  if (currentStep >= TEST_STEP_COUNT) {
    testRunning = false;
    Serial.println("MOTOR TEST FINISHED; BOTH MOTORS OFF");
    Serial.println("Send 'r' to re-enable camera driving.");
    return;
  }
  applyTestStep(currentStep);
}

static double toDegrees(const char* str, char hemi) {
  double raw = atof(str);
  int deg = (int)(raw / 100.0);
  double val = deg + (raw - deg * 100.0) / 60.0;
  return (hemi == 'S' || hemi == 'W') ? -val : val;
}

static void parseNMEA(char* s) {
  if (strncmp(s, "$GPRMC,", 7) != 0 && strncmp(s, "$GNRMC,", 7) != 0) return;
  char* star = strchr(s, '*');
  if (!star || strlen(star + 1) < 2) return;
  uint8_t checksum = 0;
  for (char* p = s + 1; p < star; ++p) checksum ^= (uint8_t)*p;
  if (checksum != (uint8_t)strtoul(star + 1, nullptr, 16)) return;
  *star = '\0';

  char* fields[13] = {};
  int n = 0;
  fields[n++] = s;
  for (char* p = s; *p && n < 13; ++p) {
    if (*p == ',') { *p = '\0'; fields[n++] = p + 1; }
  }
  if (n < 9 || fields[2][0] != 'A' || !fields[3][0] || !fields[5][0] ||
      !(fields[4][0] == 'N' || fields[4][0] == 'S') ||
      !(fields[6][0] == 'E' || fields[6][0] == 'W')) {
    gpsFix = false;
    return;
  }

  double lat = toDegrees(fields[3], fields[4][0]);
  double lon = toDegrees(fields[5], fields[6][0]);
  if (fabs(lat) > 90 || fabs(lon) > 180) return;
  gpsFix = true;
  lastFixMs = millis();
  if (!originSet) {
    lat0 = lat;
    lon0 = lon;
    originSet = true;
    Serial.println("GPS ORIGIN SET (0,0)");
  }

  const double k = 3.14159265358979323846 / 180.0;
  gpsX = (lon - lon0) * k * 6371000.0 * cos(lat0 * k);
  gpsY = (lat - lat0) * k * 6371000.0;
  ekf.update(gpsX, gpsY, 2.5);
}

static void pollGPS() {
  while (gpsPort.available()) {
    char c = (char)gpsPort.read();
    if (c == '\n') {
      nmea[nmeaLen] = '\0';
      parseNMEA(nmea);
      nmeaLen = 0;
    } else if (c != '\r') {
      if (nmeaLen < sizeof(nmea) - 1) nmea[nmeaLen++] = c;
      else nmeaLen = 0;
    }
  }
}

static void pollCamera() {
  uint8_t received = Wire.requestFrom((uint8_t)CAMERA_ADDR, (uint8_t)5);
  if (received != 5) {
    while (Wire.available()) Wire.read();
    applyCameraCommand(CMD_STOP); // Fail-safe: missing I2C response
    return;  // no blocking delay: keep IMU / EKF timing regular
  }

  uint8_t data[5];
  for (uint8_t i = 0; i < 5; ++i) data[i] = Wire.read();
  int16_t x = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  int16_t y = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  uint8_t command = data[4];

  // Invalid target position => stop. Unknown commands also map to STOP.
  if (x < 0 || y < 0) {
    applyCameraCommand(CMD_STOP);
  } else {
    applyCameraCommand(command);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  if (!beginMotors()) {
    Serial.println("ERROR: motor PWM initialization failed");
    while (true) delay(1000); // EN pins remain LOW
  }

  if (!imu.begin(21, 22)) {
    Serial.println("ERROR MPU6050 not found");
    while (true) delay(1000);
  }
  gpsPort.setRxBufferSize(2048);
  gpsPort.begin(9600, SERIAL_8N1, GPS_RX, -1);

  Serial.println("Keep IMU stationary for calibration...");
  if (!imu.calibrate()) {
    Serial.println("ERROR IMU calibration failed");
    while (true) delay(1000);
  }
  ekf.initializeAttitude(imu.getStationaryAccel());
  lastImuUs = micros();
  lastCameraMs = lastPrintMs = millis();
  Serial.println("IMU + GPS + EKF + CAMERA + TWO HW-039 READY");
  Serial.println("Camera drives motors: FORWARD/LEFT/RIGHT/STOP.");
  Serial.println("Serial: 's' = stop/lock, 'r' = resume camera, 't' = motor test.");
}

void loop() {
  pollSerialCommands();
  updateMotorTest();
  pollGPS();

  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastImuUs) >= 10000) {
    double dt = (uint32_t)(nowUs - lastImuUs) * 1e-6;
    lastImuUs = nowUs;
    if (imu.update()) ekf.predict(imu.getAccel(), imu.getGyro(), dt);
  }

  uint32_t ms = millis();
  if ((uint32_t)(ms - lastCameraMs) >= 100) {
    lastCameraMs = ms;
    pollCamera();
  }
  if ((uint32_t)(ms - lastPrintMs) >= 1000) {
    lastPrintMs = ms;
    State s = ekf.getState();
    bool fresh = gpsFix && ((uint32_t)(ms - lastFixMs) < 3000);
    Serial.printf("GPS %s | raw local X=%.2f Y=%.2f m\n",
                  fresh ? "FIX" : "NO FIX", gpsX, gpsY);
    Serial.printf("EKF position X=%.2f Y=%.2f Z=%.2f m\n",
                  s.position.x(), s.position.y(), s.position.z());
    Serial.printf("EKF velocity Vx=%.2f Vy=%.2f Vz=%.2f m/s\n",
                  s.velocity.x(), s.velocity.y(), s.velocity.z());
  }
}
