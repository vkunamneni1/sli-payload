/*
 * ============================================================================
 * PROJECT LAKSHMI — GROUND STATION
 * ============================================================================
 * Arduino Nano + LoRa Ra-02 (433MHz)
 * Connect to computer via USB. Open Serial Monitor at 9600 baud.
 * Type a letter + Enter to send command to rover.
 * Rover responses appear automatically.
 *
 * COMMANDS:
 *   S = Fire solenoid latches (release payload)
 *   D = Deploy rack & pinion (push rover out)
 *   E = Expand wheels
 *   C = Collapse wheels
 *   F = Drive forward        B = Drive backward
 *   L = Turn left             R = Turn right
 *   X = Stop motors
 *   G = FULL AUTO: solenoid -> rack -> expand -> drive 5ft
 *   O = EMERGENCY OFF (everything stops)
 * ============================================================================
 */

#include <LoRa.h>
#include <SPI.h>

#define PIN_LORA_CS 10
#define PIN_LORA_RST 8
#define PIN_LORA_DIO0 7

void setup() {
  Serial.begin(9600);
  Serial.println(F("============================="));
  Serial.println(F(" LAKSHMI GROUND STATION"));
  Serial.println(F("============================="));

  LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);
  if (LoRa.begin(433E6)) {
    LoRa.setSyncWord(0x34);
    Serial.println(F("[OK] LoRa 433MHz ready"));
  } else {
    Serial.println(F("[FAIL] LoRa not found!"));
    while (1)
      ;
  }

  Serial.println();
  Serial.println(F("---- COMMAND MENU ----"));
  Serial.println(F(" S = Solenoid (release latches)"));
  Serial.println(F(" D = Deploy rack & pinion"));
  Serial.println(F(" E = Expand wheels"));
  Serial.println(F(" C = Collapse wheels"));
  Serial.println(F(" F = Forward   B = Backward"));
  Serial.println(F(" L = Left      R = Right"));
  Serial.println(F(" X = Stop motors"));
  Serial.println(F(" G = FULL AUTO SEQUENCE"));
  Serial.println(F(" O = EMERGENCY OFF"));
  Serial.println(F("----------------------"));
  Serial.println();
}

void loop() {
  // ---- Send typed commands over LoRa ----
  if (Serial.available()) {
    char cmd = Serial.read();
    // Flush extra chars (newline, CR, etc.)
    while (Serial.available() > 0)
      Serial.read();

    // Skip newlines
    if (cmd == '\n' || cmd == '\r')
      return;

    // Convert lowercase to uppercase for consistency
    if (cmd >= 'a' && cmd <= 'z')
      cmd -= 32;

    Serial.print(F("[SENT] "));
    switch (cmd) {
    case 'S':
      Serial.println(F("Solenoid fire"));
      break;
    case 'D':
      Serial.println(F("Deploy rack"));
      break;
    case 'E':
      Serial.println(F("Expand wheels"));
      break;
    case 'C':
      Serial.println(F("Collapse wheels"));
      break;
    case 'F':
      Serial.println(F("Drive FORWARD"));
      break;
    case 'B':
      Serial.println(F("Drive BACKWARD"));
      break;
    case 'L':
      Serial.println(F("Turn LEFT"));
      break;
    case 'R':
      Serial.println(F("Turn RIGHT"));
      break;
    case 'X':
      Serial.println(F("STOP motors"));
      break;
    case 'G':
      Serial.println(F("FULL AUTO GO!"));
      break;
    case 'O':
      Serial.println(F("EMERGENCY OFF"));
      break;
    default:
      Serial.print(F("Unknown: '"));
      Serial.print(cmd);
      Serial.println(F("'"));
      return; // Don't send unknown commands
    }

    LoRa.beginPacket();
    LoRa.write(cmd);
    LoRa.endPacket();
  }

  // ---- Receive rover responses ----
  int pkt = LoRa.parsePacket();
  if (pkt > 0) {
    Serial.print(F("[ROVER] "));
    while (LoRa.available()) {
      Serial.print((char)LoRa.read());
    }
    Serial.print(F("  RSSI:"));
    Serial.print(LoRa.packetRssi());
    Serial.println(F("dBm"));
  }
}
