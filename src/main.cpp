/**
 * ============================================================================
 * PROJECT LAKSHMI — Main FSM (Arduino Nano)
 * ============================================================================
 * Mission: Detect landing -> release solenoids -> deploy via rack & pinion
 *          -> drive 1+ meter away -> done.
 * 
 * Non-blocking loop (millis-based timers, no delay()).
 * ============================================================================
 */

#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>
#include <EEPROM.h>

#include "Hardware_Config.h"
#include "Rover_HAL.h"

// ============================================================================
// GLOBALS
// ============================================================================

static RoverState state = RoverState::INIT;
static Servo deployServo;

// Timing
static unsigned long stateEntry = 0;
static unsigned long lastTelem  = 0;
static unsigned long lastIMU    = 0;

// IMU
static float gx = 0, gy = 0, gz = 0;
static float zGs = 0;

// Launch detection
static unsigned long launchStart = 0;
static bool launchHeld = false;

// Landing detection
static unsigned long landStart = 0;
static bool landStable = false;

// Traverse
static float pidI = 0, pidPrev = 0;
static unsigned long pidMs = 0;
static float targetHdg = 0;
static unsigned long travStart = 0;
static unsigned long stallMs = 0;
static bool escaping = false;
static uint8_t escPhase = 0;
static unsigned long escStart = 0;

// Telemetry
static TelemetryPacket tpkt;
static float battV = 11.1f;

// ============================================================================
// HELPERS
// ============================================================================

static void goState(RoverState s) {
    Serial.print(F("S:"));
    Serial.print((uint8_t)state);
    Serial.print('>');
    Serial.println((uint8_t)s);
    state = s;
    stateEntry = millis();
    EEPROM.update(0, (uint8_t)s);
}

static void sendTelem() {
    tpkt.state = (uint8_t)state;
    tpkt.missionTimeSec = (uint16_t)(millis() / 1000UL);
    tpkt.battMv = (int16_t)(battV * 1000.0f);
    tpkt.accelZ = (int16_t)(zGs * 100.0f);
    tpkt.heading = (int16_t)(halReadYawDeg() * 10.0f);
    tpkt.flags = 0;
    if (battV < BATT_CRITICAL_V) tpkt.flags |= 0x01;
    if (escaping) tpkt.flags |= 0x02;
    halTransmitLoRa(tpkt);
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    Serial.begin(115200);
    Serial.println(F("=== LAKSHMI v3 (Nano) ==="));

    halInitAll();

    deployServo.attach(PIN_DEPLOY_SERVO);
    deployServo.write(DEPLOY_STOWED);

    // LoRa init
    if (!LoRa.begin((long)LORA_FREQ)) {
        Serial.println(F("ERR:LoRa"));
        goState(RoverState::SAFE_MODE);
        return;
    }
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    Serial.println(F("LoRa OK"));

    // EEPROM recovery (only for post-flight states)
    uint8_t saved = EEPROM.read(0);
    if (saved >= (uint8_t)RoverState::SOLENOID_RELEASE &&
        saved <= (uint8_t)RoverState::TRAVERSE) {
        Serial.print(F("RECOVER:"));
        Serial.println(saved);
        state = (RoverState)saved;
        stateEntry = millis();
    } else {
        goState(RoverState::PAD_STANDBY);
    }

    battV = halReadBatteryVoltage();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
    unsigned long now = millis();

    // ---- IMU @ 50Hz ----
    if (now - lastIMU >= 20UL) {
        lastIMU = now;
        halReadAccelGs(gx, gy, gz);
        zGs = gz;
    }

    // ---- Battery @ 0.5Hz ----
    static unsigned long lastBatt = 0;
    if (now - lastBatt >= 2000UL) {
        lastBatt = now;
        battV = halReadBatteryVoltage();
        if (battV < BATT_CRITICAL_V && state != RoverState::SAFE_MODE) {
            halStopMotors();
            goState(RoverState::SAFE_MODE);
            return;
        }
    }

    // ---- Telemetry ----
    unsigned long tInt = (state == RoverState::PAD_STANDBY)
                         ? STANDBY_TELEM_MS : TELEM_INTERVAL_MS;
    if (now - lastTelem >= tInt) {
        lastTelem = now;
        sendTelem();
    }

    // ================================================================
    // FSM
    // ================================================================
    switch (state) {

    // ---- PAD_STANDBY: wait for launch ----
    case RoverState::PAD_STANDBY: {
        if (zGs >= LAUNCH_G_THRESH) {
            if (!launchHeld) { launchStart = now; launchHeld = true; }
            else if (now - launchStart >= LAUNCH_SUSTAIN_MS) {
                Serial.println(F("LAUNCH"));
                goState(RoverState::ASCENT);
            }
        } else {
            launchHeld = false;
        }
        break;
    }

    // ---- ASCENT: wait for apogee ----
    case RoverState::ASCENT: {
        if (zGs <= 0.0f) {
            Serial.println(F("APOGEE"));
            goState(RoverState::DESCENT);
        }
        break;
    }

    // ---- DESCENT: wait for 1G stable 10s ----
    case RoverState::DESCENT: {
        float err = zGs - LANDING_G_TARGET;
        if (err < 0) err = -err;
        if (err <= LANDING_G_TOL) {
            if (!landStable) { landStart = now; landStable = true; }
            else if (now - landStart >= LANDING_STABLE_MS) {
                Serial.println(F("LANDED"));
                goState(RoverState::LANDED_WAIT);
            }
        } else {
            landStable = false;
        }
        break;
    }

    // ---- LANDED_WAIT: 30s for parachute settling ----
    case RoverState::LANDED_WAIT: {
        if (now - stateEntry >= POST_LAND_WAIT_MS) {
            goState(RoverState::SOLENOID_RELEASE);
        }
        break;
    }

    // ---- SOLENOID_RELEASE: fire MOSFET 2s ----
    case RoverState::SOLENOID_RELEASE: {
        if (now - stateEntry < 50UL) {
            halFireSolenoid(true);
            Serial.println(F("SOL ON"));
        }
        if (now - stateEntry >= SOLENOID_PULSE_MS) {
            halFireSolenoid(false);
            Serial.println(F("SOL OFF"));
            goState(RoverState::DEPLOY_EGRESS);
        }
        break;
    }

    // ---- DEPLOY_EGRESS: rack & pinion servo extends ----
    case RoverState::DEPLOY_EGRESS: {
        if (now - stateEntry < 50UL) {
            deployServo.write(DEPLOY_EXTENDED);
            Serial.println(F("DEPLOY"));
        }
        if (now - stateEntry >= DEPLOY_MOVE_MS) {
            Serial.println(F("OUT"));
            goState(RoverState::TRAVERSE);
        }
        break;
    }

    // ---- TRAVERSE: PID heading hold, drive 1+ meter ----
    case RoverState::TRAVERSE: {
        // Init on entry
        if (travStart == 0) {
            travStart = now;
            targetHdg = halReadYawDeg();
            pidI = 0;
            pidPrev = 0;
            pidMs = now;
            stallMs = now;
            Serial.print(F("HDG:"));
            Serial.println(targetHdg);
        }

        // -- Rut escape --
        if (escaping) {
            if (escPhase == 0) {
                halDriveMotors(-200, -200);
                if (now - escStart >= ESCAPE_REV_MS) {
                    escPhase = 1;
                    escStart = now;
                }
            } else {
                halDriveMotors(180, -180);
                if (now - escStart >= ESCAPE_TURN_MS) {
                    escaping = false;
                    escPhase = 0;
                    targetHdg = halReadYawDeg();
                    pidI = 0;
                    stallMs = now;
                }
            }
            break;
        }

        // -- Stall detect --
        if (now - stallMs >= STALL_TIMEOUT_MS) {
            stallMs = now;
            float absAx = gx;
            if (absAx < 0) absAx = -absAx;
            if (absAx < STALL_ACCEL_MIN) {
                Serial.println(F("STALL"));
                escaping = true;
                escPhase = 0;
                escStart = now;
                break;
            }
        }

        // -- PID drive --
        float yaw = halReadYawDeg();
        float corr = halPIDCompute(targetHdg, yaw, pidI, pidPrev, pidMs);
        int16_t lp = (int16_t)TRAVERSE_PWM + (int16_t)corr;
        int16_t rp = (int16_t)TRAVERSE_PWM - (int16_t)corr;
        halDriveMotors(lp, rp);

        // -- Done --
        if (now - travStart >= TRAVERSE_TIME_MS) {
            halStopMotors();
            travStart = 0;
            Serial.println(F("TRAV DONE"));
            goState(RoverState::MISSION_COMPLETE);
        }
        break;
    }

    // ---- MISSION_COMPLETE ----
    case RoverState::MISSION_COMPLETE:
        halStopMotors();
        break;

    // ---- SAFE_MODE ----
    case RoverState::SAFE_MODE:
        halStopMotors();
        halFireSolenoid(false);
        break;

    case RoverState::INIT:
    default:
        goState(RoverState::SAFE_MODE);
        break;
    }
}
