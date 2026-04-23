/**
 * ============================================================================
 * PROJECT LAKSHMI — BENCH TEST MODE
 * ============================================================================
 * Test the solenoid release + rack & pinion deployment module.
 * No IMU, no motors, no flight detection.
 * 
 * WIRING (matches your diagram):
 *   D9:  Solenoid MOSFET gate
 *   D10: LoRa CS    D11: MOSI    D12: MISO    D13: SCK
 *   D8:  LoRa RST   D2: LoRa DIO0
 *   A2:  Deployment servo (rack & pinion)
 * 
 * COMMANDS (type in Serial Monitor at 115200 baud, send with newline):
 *   s  = Fire solenoid (holds 2 seconds, then releases)
 *   d  = Deploy rack & pinion (servo to 180)
 *   r  = Retract rack & pinion (servo to 0)
 *   t  = Send a test LoRa packet
 *   f  = Full sequence: solenoid -> wait -> deploy
 *   h  = Print help
 * ============================================================================
 */

#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>

// ---- Pin definitions (from your wiring diagram) ----
#define PIN_SOLENOID      9
#define PIN_LORA_CS       10
#define PIN_LORA_RST      8
#define PIN_LORA_DIO0     2
#define PIN_DEPLOY_SERVO  A2

// ---- Servo positions ----
#define SERVO_STOWED      0
#define SERVO_DEPLOYED    180

// ---- Solenoid pulse time ----
#define SOL_PULSE_MS      2000UL

// ---- Forward declarations ----
void handleCommand(char cmd);
void runSequence(unsigned long now);
void printHelp();

// ---- Globals ----
static Servo deployServo;
static bool loraOK = false;

// Non-blocking solenoid state
static bool solFiring = false;
static unsigned long solStart = 0;

// Non-blocking full sequence
static bool runningSequence = false;
static uint8_t seqStep = 0;
static unsigned long seqTimer = 0;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    
    Serial.println(F("================================"));
    Serial.println(F(" LAKSHMI BENCH TEST"));
    Serial.println(F(" Solenoid + Rack & Pinion Module"));
    Serial.println(F("================================"));

    // Solenoid MOSFET
    pinMode(PIN_SOLENOID, OUTPUT);
    digitalWrite(PIN_SOLENOID, LOW);
    Serial.println(F("[OK] Solenoid pin D9 (LOW)"));

    // Deployment servo
    deployServo.attach(PIN_DEPLOY_SERVO);
    deployServo.write(SERVO_STOWED);
    Serial.println(F("[OK] Servo on A2 (stowed at 0)"));

    // LoRa
    LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);
    if (LoRa.begin(433E6)) {
        LoRa.setTxPower(17);
        LoRa.setSyncWord(0x34);
        loraOK = true;
        Serial.println(F("[OK] LoRa 433MHz"));
    } else {
        Serial.println(F("[FAIL] LoRa not detected!"));
    }

    Serial.println();
    printHelp();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
    unsigned long now = millis();

    // ---- Handle non-blocking solenoid pulse ----
    if (solFiring && (now - solStart >= SOL_PULSE_MS)) {
        digitalWrite(PIN_SOLENOID, LOW);
        solFiring = false;
        Serial.println(F("[SOL] OFF (2s pulse done)"));
    }

    // ---- Handle full sequence ----
    if (runningSequence) {
        runSequence(now);
    }

    // ---- Process serial commands ----
    if (Serial.available()) {
        char cmd = Serial.read();
        // Flush remaining chars (newline, etc.)
        while (Serial.available()) Serial.read();
        
        handleCommand(cmd);
    }
}

// ============================================================================
// COMMAND HANDLER
// ============================================================================

void handleCommand(char cmd) {
    switch (cmd) {
    case 's':
    case 'S':
        Serial.println(F("\n--- SOLENOID TEST ---"));
        Serial.println(F("[SOL] Firing MOSFET on D9..."));
        digitalWrite(PIN_SOLENOID, HIGH);
        solFiring = true;
        solStart = millis();
        Serial.println(F("[SOL] ON — will auto-off in 2 seconds"));
        break;

    case 'd':
    case 'D':
        Serial.println(F("\n--- DEPLOY TEST ---"));
        deployServo.write(SERVO_DEPLOYED);
        Serial.println(F("[SERVO] Moving to 180 (deployed)"));
        break;

    case 'r':
    case 'R':
        Serial.println(F("\n--- RETRACT TEST ---"));
        deployServo.write(SERVO_STOWED);
        Serial.println(F("[SERVO] Moving to 0 (stowed)"));
        break;

    case 't':
    case 'T':
        Serial.println(F("\n--- LoRa TX TEST ---"));
        if (loraOK) {
            LoRa.beginPacket();
            LoRa.print(F("LAKSHMI TEST"));
            LoRa.endPacket();
            Serial.println(F("[LoRa] Sent 'LAKSHMI TEST'"));
        } else {
            Serial.println(F("[LoRa] FAILED — not initialized"));
        }
        break;

    case 'f':
    case 'F':
        Serial.println(F("\n--- FULL SEQUENCE ---"));
        Serial.println(F("Step 1: Solenoid fire (2s)"));
        Serial.println(F("Step 2: Wait 1s"));
        Serial.println(F("Step 3: Deploy servo (3s)"));
        Serial.println(F("Starting..."));
        runningSequence = true;
        seqStep = 0;
        seqTimer = millis();
        break;

    case 'h':
    case 'H':
        printHelp();
        break;

    case '\n':
    case '\r':
        break;  // ignore newlines

    default:
        Serial.print(F("Unknown: '"));
        Serial.print(cmd);
        Serial.println(F("' — type 'h' for help"));
        break;
    }
}

// ============================================================================
// FULL SEQUENCE (non-blocking)
// ============================================================================

void runSequence(unsigned long now) {
    switch (seqStep) {
    case 0:
        // Fire solenoid
        digitalWrite(PIN_SOLENOID, HIGH);
        Serial.println(F("[SEQ] Solenoid ON"));
        seqStep = 1;
        seqTimer = now;
        break;

    case 1:
        // Wait 2s for solenoid
        if (now - seqTimer >= 2000UL) {
            digitalWrite(PIN_SOLENOID, LOW);
            Serial.println(F("[SEQ] Solenoid OFF"));
            seqStep = 2;
            seqTimer = now;
        }
        break;

    case 2:
        // Wait 1s pause
        if (now - seqTimer >= 1000UL) {
            Serial.println(F("[SEQ] Deploying servo..."));
            deployServo.write(SERVO_DEPLOYED);
            seqStep = 3;
            seqTimer = now;
        }
        break;

    case 3:
        // Wait 3s for servo
        if (now - seqTimer >= 3000UL) {
            Serial.println(F("[SEQ] === SEQUENCE COMPLETE ==="));
            Serial.println(F("[SEQ] Type 'r' to retract servo"));
            runningSequence = false;
            seqStep = 0;
        }
        break;
    }
}

// ============================================================================
// HELP
// ============================================================================

void printHelp() {
    Serial.println(F("--- COMMANDS ---"));
    Serial.println(F("  s = Fire solenoid (2s pulse)"));
    Serial.println(F("  d = Deploy servo (180)"));
    Serial.println(F("  r = Retract servo (0)"));
    Serial.println(F("  t = LoRa test TX"));
    Serial.println(F("  f = Full sequence (sol->deploy)"));
    Serial.println(F("  h = This help"));
    Serial.println(F("----------------"));
}
