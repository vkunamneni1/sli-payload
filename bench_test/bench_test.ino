/*
 * ============================================================
 * PROJECT LAKSHMI — BENCH TEST
 * ============================================================
 * Open in Arduino IDE. Select:
 *   Board:  Arduino Nano
 *   Processor: ATmega328P (Old Bootloader)  ← try this first
 *   Port:   your COM port
 *
 * If upload fails, switch Processor to "ATmega328P" (no "Old")
 *
 * Then open Serial Monitor at 115200 baud and type commands.
 *
 * WIRING:
 *   D9:  Solenoid MOSFET gate
 *   D10: LoRa CS    D11: MOSI    D12: MISO    D13: SCK
 *   D8:  LoRa RST   D2:  LoRa DIO0
 *   A2:  Deployment servo (rack & pinion)
 *
 * COMMANDS (type in Serial Monitor, press Enter):
 *   s = Fire solenoid (2 second pulse)
 *   d = Deploy servo to 180 (rack extends)
 *   r = Retract servo to 0 (rack retracts)
 *   t = Send test LoRa packet
 *   f = Full sequence: solenoid -> pause -> deploy
 *   h = Help
 * ============================================================
 */

#include <LoRa.h>
#include <SPI.h>
#include <Servo.h>

// ---- PINS (match your wiring diagram) ----
#define PIN_SOLENOID 9
#define PIN_LORA_CS 10
#define PIN_LORA_RST 8
#define PIN_LORA_DIO0 2
#define PIN_DEPLOY_SERVO A2

// ---- SETTINGS ----
#define SERVO_STOWED 0
#define SERVO_DEPLOYED 180
#define SOL_PULSE_MS 2000

// ---- GLOBALS ----
Servo deployServo;
bool loraOK = false;

// Solenoid timing
bool solFiring = false;
unsigned long solStart = 0;

// Full sequence
bool seqRunning = false;
byte seqStep = 0;
unsigned long seqTimer = 0;

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println(F("================================"));
  Serial.println(F(" LAKSHMI BENCH TEST"));
  Serial.println(F("================================"));

  // Solenoid
  pinMode(PIN_SOLENOID, OUTPUT);
  digitalWrite(PIN_SOLENOID, LOW);
  Serial.println(F("[OK] Solenoid D9"));

  // Servo
  deployServo.attach(PIN_DEPLOY_SERVO);
  deployServo.write(SERVO_STOWED);
  Serial.println(F("[OK] Servo A2"));

  // LoRa
  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);
  if (LoRa.begin(433E6)) {
    LoRa.setTxPower(17);
    LoRa.setSyncWord(0x34);
    loraOK = true;
    Serial.println(F("[OK] LoRa 433MHz"));
  } else {
    Serial.println(F("[FAIL] LoRa!"));
  }

  Serial.println();
  Serial.println(
      F("COMMANDS: s=solenoid d=deploy r=retract t=lora f=full h=help"));
  Serial.println(F("Type a letter and press Enter."));
  Serial.println();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  unsigned long now = millis();

  // Auto-off solenoid after 2s
  if (solFiring && (now - solStart >= SOL_PULSE_MS)) {
    digitalWrite(PIN_SOLENOID, LOW);
    solFiring = false;
    Serial.println(F("[SOL] OFF"));
  }

  // Full sequence state machine
  if (seqRunning) {
    switch (seqStep) {
    case 0:
      digitalWrite(PIN_SOLENOID, HIGH);
      Serial.println(F("[SEQ] Solenoid ON"));
      seqStep = 1;
      seqTimer = now;
      break;
    case 1:
      if (now - seqTimer >= 2000) {
        digitalWrite(PIN_SOLENOID, LOW);
        Serial.println(F("[SEQ] Solenoid OFF"));
        seqStep = 2;
        seqTimer = now;
      }
      break;
    case 2:
      if (now - seqTimer >= 1000) {
        deployServo.write(SERVO_DEPLOYED);
        Serial.println(F("[SEQ] Deploying servo..."));
        seqStep = 3;
        seqTimer = now;
      }
      break;
    case 3:
      if (now - seqTimer >= 3000) {
        Serial.println(F("[SEQ] DONE! Type 'r' to retract."));
        seqRunning = false;
        seqStep = 0;
      }
      break;
    }
  }

  // Read serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available())
      Serial.read(); // flush

    switch (cmd) {
    case 's':
    case 'S':
      Serial.println(F("--- SOLENOID ---"));
      digitalWrite(PIN_SOLENOID, HIGH);
      solFiring = true;
      solStart = millis();
      Serial.println(F("[SOL] ON (2s pulse)"));
      break;

    case 'd':
    case 'D':
      Serial.println(F("--- DEPLOY ---"));
      deployServo.write(SERVO_DEPLOYED);
      Serial.println(F("[SERVO] -> 180"));
      break;

    case 'r':
    case 'R':
      Serial.println(F("--- RETRACT ---"));
      deployServo.write(SERVO_STOWED);
      Serial.println(F("[SERVO] -> 0"));
      break;

    case 't':
    case 'T':
      Serial.println(F("--- LoRa TX ---"));
      if (loraOK) {
        LoRa.beginPacket();
        LoRa.print("LAKSHMI TEST");
        LoRa.endPacket();
        Serial.println(F("[LoRa] Sent!"));
      } else {
        Serial.println(F("[LoRa] Not connected"));
      }
      break;

    case 'f':
    case 'F':
      Serial.println(F("--- FULL SEQUENCE ---"));
      Serial.println(F("Sol 2s -> pause 1s -> deploy"));
      seqRunning = true;
      seqStep = 0;
      seqTimer = millis();
      break;

    case 'h':
    case 'H':
      Serial.println(F("s=solenoid d=deploy r=retract t=lora f=full"));
      break;
    }
  }
}
