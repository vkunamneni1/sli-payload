/*
 * ============================================================================
 * PROJECT LAKSHMI — RELEASE MODULE (Solenoid & Rack Controller)
 * ============================================================================
 * Arduino Nano + LoRa Ra-02 (433MHz)
 * Placed in the launch vehicle payload bay.
 * Controls the solenoid latches and the rack & pinion deployment servo.
 *
 * PINS:
 *   D6 = Solenoid MOSFET Gate
 *   D3 = Rack & Pinion Servo
 *   D7, D8, D10-13 = LoRa SPI
 *
 * COMMANDS RECEIVED:
 *   S = Fire solenoid for 5 seconds
 *   D = Deploy rack & pinion
 *   Q = Retract rack & pinion
 *   O = Emergency Off
 * ============================================================================
 */

#include <LoRa.h>
#include <SPI.h>
#include <Servo.h>

#define PIN_LORA_CS       10
#define PIN_LORA_RST       8
#define PIN_LORA_DIO0      7

#define PIN_SOLENOID       6
#define PIN_RACK_SERVO     3

#define SOL_PULSE_MS      5000
#define RACK_STOWED          0
#define RACK_EXTENDED      180

Servo rackServo;
bool rackAttached = false;

bool solFiring = false;
unsigned long solStart = 0;

void sendReply(const char* msg) {
  LoRa.beginPacket();
  LoRa.print(F("RM:"));
  LoRa.print(msg);
  LoRa.endPacket(true);
}

void attachRack() {
  if (!rackAttached) {
    rackServo.attach(PIN_RACK_SERVO);
    rackAttached = true;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println(F("=== LAKSHMI RELEASE MODULE ==="));

  pinMode(PIN_SOLENOID, OUTPUT);
  digitalWrite(PIN_SOLENOID, LOW);

  pinMode(PIN_RACK_SERVO, OUTPUT);
  digitalWrite(PIN_RACK_SERVO, LOW);

  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);
  if (LoRa.begin(433E6)) {
    LoRa.setSyncWord(0x34);
    Serial.println(F("[OK] LoRa 433MHz"));
  } else {
    Serial.println(F("[FAIL] LoRa!"));
  }
}

void loop() {
  unsigned long now = millis();

  // Non-blocking solenoid pulse
  if (solFiring && (now - solStart >= SOL_PULSE_MS)) {
    digitalWrite(PIN_SOLENOID, LOW);
    solFiring = false;
    Serial.println(F("[SOL] Done"));
    sendReply("SOL_DONE");
  }

  // Check LoRa commands
  int pkt = LoRa.parsePacket();
  if (pkt > 0) {
    char cmd = 0;
    while (LoRa.available()) cmd = (char)LoRa.read();
    
    if (cmd >= 'a' && cmd <= 'z') cmd -= 32;

    switch (cmd) {
      case 'S':
        Serial.println(F("[CMD] Solenoid Fire"));
        digitalWrite(PIN_SOLENOID, HIGH);
        solFiring = true;
        solStart = now;
        sendReply("SOL_ON");
        break;
      
      case 'D':
        Serial.println(F("[CMD] Deploy Rack"));
        attachRack();
        rackServo.write(RACK_EXTENDED);
        sendReply("RACK_OUT");
        break;

      case 'Q':
        Serial.println(F("[CMD] Retract Rack"));
        attachRack();
        rackServo.write(RACK_STOWED);
        sendReply("RACK_IN");
        break;

      case 'O':
        Serial.println(F("[CMD] E-STOP"));
        digitalWrite(PIN_SOLENOID, LOW);
        solFiring = false;
        if (rackAttached) { rackServo.detach(); rackAttached = false; }
        sendReply("STOPPED");
        break;
    }
  }
}
