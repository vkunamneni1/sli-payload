/*
 * PROJECT LAKSHMI — ROVER (LoRa commands ONLY, no serial input)
 * 
 * Pins: D6=Solenoid, D3=Servo, D10=LoRa CS, D8=LoRa RST, D7=LoRa DIO0
 * All commands come from LoRa ground station only.
 * Serial Monitor is debug output only.
 */

#include <LoRa.h>
#include <SPI.h>
#include <Servo.h>

// ---- PINS ----
#define PIN_SOLENOID 6
#define PIN_DEPLOY_SERVO 3
#define PIN_LORA_CS 10
#define PIN_LORA_RST 8
#define PIN_LORA_DIO0 7

// ---- SETTINGS ----
#define SERVO_STOWED 0
#define SERVO_DEPLOYED 180
#define SOL_PULSE_MS 5000
#define LORA_FREQ 433E6

// ---- GLOBALS ----
Servo deployServo;
bool servoAttached = false;  // Don't attach until commanded
bool loraOK = false;

bool solFiring = false;
unsigned long solStart = 0;

bool seqRunning = false;
byte seqStep = 0;
unsigned long seqTimer = 0;

void handleCommand(char cmd);
void sendLoRaStatus(const char* msg);
void attachServoSafe();

// ============================================================
void setup() {
  Serial.begin(9600);
  Serial.println(F("--- LAKSHMI ROVER (LoRa only) ---"));

  pinMode(PIN_SOLENOID, OUTPUT);
  digitalWrite(PIN_SOLENOID, LOW);

  // Do NOT attach servo on boot — prevents auto-movement
  pinMode(PIN_DEPLOY_SERVO, OUTPUT);
  digitalWrite(PIN_DEPLOY_SERVO, LOW);

  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);
  if (LoRa.begin((long)LORA_FREQ)) {
    LoRa.setSyncWord(0x34);
    loraOK = true;
    Serial.println(F("[OK] LoRa 433MHz"));
  } else {
    Serial.println(F("[FAIL] LoRa!"));
  }

  Serial.println(F("Waiting for LoRa commands..."));
}

// ============================================================
void loop() {
  unsigned long now = millis();

  // ---- Auto-off solenoid ----
  if (solFiring && (now - solStart >= SOL_PULSE_MS)) {
    digitalWrite(PIN_SOLENOID, LOW);
    solFiring = false;
    Serial.println(F("[SOL] Pulse done"));
    sendLoRaStatus("SOL_OFF");
  }

  // ---- Full sequence ----
  if (seqRunning) {
    switch (seqStep) {
    case 0:
      digitalWrite(PIN_SOLENOID, HIGH);
      Serial.println(F("[SEQ] Solenoid ON"));
      sendLoRaStatus("SEQ_SOL_ON");
      seqStep = 1;
      seqTimer = now;
      break;
    case 1:
      if (now - seqTimer >= SOL_PULSE_MS) {
        digitalWrite(PIN_SOLENOID, LOW);
        Serial.println(F("[SEQ] Solenoid OFF"));
        seqStep = 2;
        seqTimer = now;
      }
      break;
    case 2:
      if (now - seqTimer >= 1000) {
        attachServoSafe();
        deployServo.write(SERVO_DEPLOYED);
        Serial.println(F("[SEQ] Servo DEPLOYED"));
        sendLoRaStatus("SEQ_DEPLOYED");
        seqStep = 3;
        seqTimer = now;
      }
      break;
    case 3:
      if (now - seqTimer >= 1000) {
        Serial.println(F("[SEQ] Complete"));
        sendLoRaStatus("SEQ_DONE");
        seqRunning = false;
        seqStep = 0;
      }
      break;
    }
  }

  // ---- LoRa receive ONLY ----
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    char cmd = 0;
    while (LoRa.available()) {
      cmd = (char)LoRa.read();
    }
    // Only accept valid command characters
    if (cmd == 's' || cmd == 'S' ||
        cmd == 'd' || cmd == 'D' ||
        cmd == 'r' || cmd == 'R' ||
        cmd == 'o' || cmd == 'O' ||
        cmd == 'f' || cmd == 'F' ||
        cmd == '?') {
      Serial.print(F("[LoRa] cmd='"));
      Serial.print(cmd);
      Serial.print(F("' RSSI="));
      Serial.println(LoRa.packetRssi());
      handleCommand(cmd);
    }
  }
}

// ============================================================
void attachServoSafe() {
  if (!servoAttached) {
    deployServo.attach(PIN_DEPLOY_SERVO);
    servoAttached = true;
  }
}

// ============================================================
void handleCommand(char cmd) {
  switch (cmd) {
  case 's':
  case 'S':
    Serial.println(F("[CMD] Solenoid"));
    digitalWrite(PIN_SOLENOID, HIGH);
    solFiring = true;
    solStart = millis();
    sendLoRaStatus("SOL_ON");
    break;

  case 'd':
  case 'D':
    Serial.println(F("[CMD] Deploy"));
    attachServoSafe();
    deployServo.write(SERVO_DEPLOYED);
    sendLoRaStatus("DEPLOYED");
    break;

  case 'r':
  case 'R':
    Serial.println(F("[CMD] Retract"));
    attachServoSafe();
    deployServo.write(SERVO_STOWED);
    sendLoRaStatus("RETRACTED");
    break;

  case 'o':
  case 'O':
    Serial.println(F("[CMD] All OFF"));
    digitalWrite(PIN_SOLENOID, LOW);
    solFiring = false;
    if (servoAttached) {
      deployServo.write(SERVO_STOWED);
      deployServo.detach();
      servoAttached = false;
    }
    sendLoRaStatus("ALL_OFF");
    break;

  case 'f':
  case 'F':
    if (!seqRunning) {
      Serial.println(F("[CMD] Full sequence"));
      seqRunning = true;
      seqStep = 0;
      sendLoRaStatus("SEQ_START");
    }
    break;

  case '?':
    sendLoRaStatus(solFiring ? "STATUS:SOL_ON" : "STATUS:IDLE");
    break;
  }
}

// ============================================================
void sendLoRaStatus(const char* msg) {
  if (!loraOK) return;
  LoRa.beginPacket();
  LoRa.print("LK:");
  LoRa.print(msg);
  LoRa.endPacket(true);
}