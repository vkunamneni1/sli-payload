/*
 * ============================================================================
 * PROJECT LAKSHMI — ROVER FLIGHT CODE
 * Edison Academy Magnet School | 2026 NASA SLI
 * ============================================================================
 * Arduino Nano + LoRa Ra-02 (433MHz)
 * 
 * PINS:
 *   D2,D4,D5 = Left motor (IN1,IN2,ENA)
 *   A0,A1,D9 = Right motor (IN3,IN4,ENB)
 *   A2       = Expanding wheel servo
 *   A6       = Soil moisture sensor
 *   A7       = Nitrate sensor
 *   D7,D8,D10-13 = LoRa SPI
 *
 * COMMANDS:
 *   E = Expand wheels   C = Collapse
 *   F = Forward         B = Backward       L = Left     R = Right
 *   X = Stop            A = Rover Auto     O = Emergency off
 *   T = Telemetry Request
 * ============================================================================
 */

#include <LoRa.h>
#include <SPI.h>
#include <Servo.h>

#define PIN_LORA_CS       10
#define PIN_LORA_RST       8
#define PIN_LORA_DIO0      7

#define PIN_WHEEL_SERVO   A2

#define PIN_ML_EN          5
#define PIN_ML_IN1         2
#define PIN_ML_IN2         4
#define PIN_MR_EN          9
#define PIN_MR_IN1        A0
#define PIN_MR_IN2        A1

#define PIN_SOIL_MOISTURE A6
#define PIN_SOIL_NITRATE  A7

// Tuning
#define WHEEL_STOW          10
#define WHEEL_EXPAND       170
#define SWEEP_MS          2000

#define DRIVE_SPEED        180
#define TURN_SPEED         150
#define SLOW_SPEED         120
#define DRIVE_5FT_MS      8000
#define CLEAR_BAY_MS      3000

#define WATCHDOG_MS       2000

// State
Servo wheelServo;
bool wheelOn = false;

bool sweepActive = false;
int sweepFrom = 0, sweepTo = 0, sweepNow = WHEEL_STOW;
unsigned long sweepStart = 0;

bool motorsRunning = false;
unsigned long lastDriveCmd = 0;

enum AutoStep : uint8_t {
  A_OFF = 0,
  A_CLEAR_BAY,
  A_CLEAR_WAIT,
  A_SWEEP,
  A_SWEEP_WAIT,
  A_DRIVE,
  A_DRIVE_WAIT,
  A_SAMPLE,
  A_DONE
};
AutoStep autoStep = A_OFF;
unsigned long autoTimer = 0;

// Helpers
void reply(const char* msg) {
  LoRa.beginPacket();
  LoRa.print(F("RV:"));
  LoRa.print(msg);
  LoRa.endPacket(true);
}

void motorsStop() {
  analogWrite(PIN_ML_EN, 0);
  analogWrite(PIN_MR_EN, 0);
  digitalWrite(PIN_ML_IN1, LOW); digitalWrite(PIN_ML_IN2, LOW);
  digitalWrite(PIN_MR_IN1, LOW); digitalWrite(PIN_MR_IN2, LOW);
  motorsRunning = false;
}

void motorsDrive(bool leftFwd, bool rightFwd, uint8_t speed) {
  if (wheelOn) { wheelServo.detach(); wheelOn = false; }
  digitalWrite(PIN_ML_IN1, leftFwd ? HIGH : LOW);
  digitalWrite(PIN_ML_IN2, leftFwd ? LOW : HIGH);
  digitalWrite(PIN_MR_IN1, rightFwd ? HIGH : LOW);
  digitalWrite(PIN_MR_IN2, rightFwd ? LOW : HIGH);
  analogWrite(PIN_ML_EN, speed);
  analogWrite(PIN_MR_EN, speed);
  motorsRunning = true;
  lastDriveCmd = millis();
}

void startSweep(int from, int to) {
  if (sweepActive) return;
  motorsStop();
  if (!wheelOn) { wheelServo.attach(PIN_WHEEL_SERVO); wheelOn = true; }
  sweepFrom = from; sweepTo = to; sweepStart = millis(); sweepActive = true;
  wheelServo.write(from);
}

void sendTelemetry() {
  int m = analogRead(PIN_SOIL_MOISTURE);
  int n = analogRead(PIN_SOIL_NITRATE);
  LoRa.beginPacket();
  LoRa.print(F("RV_TEL:"));
  LoRa.print(sweepNow); LoRa.print(F(","));
  LoRa.print(m); LoRa.print(F(","));
  LoRa.print(n);
  LoRa.endPacket(true);
}

// Auto FSM
void runAuto() {
  unsigned long now = millis();
  switch (autoStep) {
    case A_CLEAR_BAY:
      Serial.println(F("[AUTO] Clearing bay"));
      motorsDrive(true, true, SLOW_SPEED);
      reply("A:EGRESS");
      autoStep = A_CLEAR_WAIT;
      autoTimer = now;
      break;

    case A_CLEAR_WAIT:
      if (now - autoTimer >= CLEAR_BAY_MS) {
        motorsStop();
        autoStep = A_SWEEP;
        autoTimer = now;
      }
      break;

    case A_SWEEP:
      if (now - autoTimer >= 500UL) {
        startSweep(WHEEL_STOW, WHEEL_EXPAND);
        reply("A:EXPAND");
        autoStep = A_SWEEP_WAIT;
      }
      break;

    case A_SWEEP_WAIT:
      if (!sweepActive) {
        autoStep = A_DRIVE;
        autoTimer = now;
      }
      break;

    case A_DRIVE:
      if (now - autoTimer >= 500UL) {
        Serial.println(F("[AUTO] Driving 5 feet"));
        motorsDrive(true, true, DRIVE_SPEED);
        reply("A:DRIVING");
        autoStep = A_DRIVE_WAIT;
        autoTimer = now;
      }
      break;

    case A_DRIVE_WAIT:
      if (now - autoTimer >= DRIVE_5FT_MS) {
        motorsStop();
        autoStep = A_SAMPLE;
        autoTimer = now;
      }
      break;

    case A_SAMPLE:
      if (now - autoTimer >= 500UL) {
        Serial.println(F("[AUTO] Sampling"));
        sendTelemetry();
        autoStep = A_DONE;
      }
      break;

    case A_DONE:
    case A_OFF:
    default:
      break;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("=== LAKSHMI ROVER MODULE ==="));

  pinMode(PIN_ML_EN, OUTPUT); pinMode(PIN_ML_IN1, OUTPUT); pinMode(PIN_ML_IN2, OUTPUT);
  pinMode(PIN_MR_EN, OUTPUT); pinMode(PIN_MR_IN1, OUTPUT); pinMode(PIN_MR_IN2, OUTPUT);
  motorsStop();

  pinMode(PIN_WHEEL_SERVO, OUTPUT); digitalWrite(PIN_WHEEL_SERVO, LOW);

  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);
  if (LoRa.begin(433E6)) {
    LoRa.setSyncWord(0x34);
    Serial.println(F("[OK] LoRa 433MHz"));
  }
}

void loop() {
  unsigned long now = millis();

  if (sweepActive) {
    unsigned long elapsed = now - sweepStart;
    if (elapsed >= SWEEP_MS) {
      wheelServo.write(sweepTo);
      sweepNow = sweepTo; sweepActive = false;
      if (wheelOn) { wheelServo.detach(); wheelOn = false; }
      reply(sweepTo == WHEEL_EXPAND ? "WHEEL_OUT" : "WHEEL_IN");
    } else {
      float pct = (float)elapsed / (float)SWEEP_MS;
      sweepNow = sweepFrom + (int)((float)(sweepTo - sweepFrom) * pct);
      wheelServo.write(sweepNow);
    }
  }

  if (motorsRunning && autoStep == A_OFF) {
    if (now - lastDriveCmd >= WATCHDOG_MS) {
      motorsStop(); reply("WDG_STOP");
    }
  }

  if (autoStep != A_OFF) runAuto();

  int pkt = LoRa.parsePacket();
  if (pkt > 0) {
    char cmd = 0;
    while (LoRa.available()) cmd = (char)LoRa.read();
    if (cmd >= 'a' && cmd <= 'z') cmd -= 32;

    switch (cmd) {
      case 'E': startSweep(sweepNow, WHEEL_EXPAND); reply("CMD:EXP"); break;
      case 'C': startSweep(sweepNow, WHEEL_STOW); reply("CMD:COL"); break;
      case 'F': motorsDrive(true, true, DRIVE_SPEED); reply("CMD:FWD"); break;
      case 'B': motorsDrive(false, false, DRIVE_SPEED); reply("CMD:BWD"); break;
      case 'L': motorsDrive(false, true, TURN_SPEED); reply("CMD:LFT"); break;
      case 'R': motorsDrive(true, false, TURN_SPEED); reply("CMD:RHT"); break;
      case 'X': motorsStop(); reply("CMD:STP"); break;
      case 'T': sendTelemetry(); break;
      case 'A': autoStep = A_CLEAR_BAY; autoTimer = now; reply("CMD:AUTO"); break;
      case 'O': motorsStop(); sweepActive = false; autoStep = A_OFF; reply("CMD:OFF"); break;
    }
  }
}
