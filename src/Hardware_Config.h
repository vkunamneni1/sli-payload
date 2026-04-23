/**
 * ============================================================================
 * PROJECT LAKSHMI — NASA USLI
 * Hardware_Config.h — Arduino Nano (ATmega328P)
 * ============================================================================
 * MISSION: Deploy from rocket, drive 1+ meter away. That's it.
 * 
 * PIN MAP:
 *   D2:  LoRa DIO0         D3:  Left motor PWM
 *   D4:  Left motor DIR    D5:  Right motor PWM
 *   D6:  Right motor DIR   D8:  LoRa RST
 *   D9:  Solenoid MOSFET   D10: LoRa CS
 *   D11: MOSI              D12: MISO
 *   D13: SCK               A2:  Deploy servo
 *   A4:  I2C SDA           A5:  I2C SCL
 *   A6:  Battery voltage
 * ============================================================================
 */

#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H

#include <Arduino.h>

// ============================================================================
// FSM STATES
// ============================================================================
enum class RoverState : uint8_t {
    INIT              = 0,
    PAD_STANDBY       = 1,
    ASCENT            = 2,
    DESCENT           = 3,
    LANDED_WAIT       = 4,
    SOLENOID_RELEASE  = 5,
    DEPLOY_EGRESS     = 6,
    TRAVERSE          = 7,
    MISSION_COMPLETE  = 8,
    SAFE_MODE         = 9
};

// ============================================================================
// PINS — LoRa Ra-02 433 MHz (SPI)
// ============================================================================
#define PIN_LORA_CS       10
#define PIN_LORA_RST       8
#define PIN_LORA_DIO0      2

// ============================================================================
// PINS — Motors (GoBilda 435 RPM 12V, via H-bridge)
// ============================================================================
#define PIN_LEFT_PWM       3
#define PIN_LEFT_DIR       4
#define PIN_RIGHT_PWM      5
#define PIN_RIGHT_DIR      6

// ============================================================================
// PINS — Solenoid latch release (MOSFET gate)
// ============================================================================
#define PIN_SOLENOID       9

// ============================================================================
// PINS — Deployment servo (rack & pinion)
// ============================================================================
#define PIN_DEPLOY_SERVO  A2

// ============================================================================
// PINS — IMU (MPU6050 on I2C: A4=SDA, A5=SCL)
// ============================================================================
#define MPU6050_ADDR      0x68

// ============================================================================
// PINS — Battery (analog-only pin)
// ============================================================================
#define PIN_BATT          A6
#define VDIV_RATIO        0.3197f   // 10k/4.7k divider
#define BATT_CRITICAL_V   9.6f

// ============================================================================
// LoRa
// ============================================================================
#define LORA_FREQ         433E6
#define LORA_TX_POWER     17
#define LORA_SYNC_WORD    0x34

// ============================================================================
// IMU THRESHOLDS
// ============================================================================
#define LAUNCH_G_THRESH   4.0f
#define LAUNCH_SUSTAIN_MS 200UL
#define LANDING_G_TARGET  1.0f
#define LANDING_G_TOL     0.15f
#define LANDING_STABLE_MS 10000UL  // 10s at 1G
#define POST_LAND_WAIT_MS 30000UL  // 30s parachute settling

// ============================================================================
// DRIVE
// ============================================================================
#define TRAVERSE_PWM      180      // ~70%
#define TRAVERSE_TIME_MS  8000UL   // 8s ~ 1+ meter
#define STALL_TIMEOUT_MS  2000UL
#define STALL_ACCEL_MIN   0.3f
#define ESCAPE_REV_MS     1500UL
#define ESCAPE_TURN_MS    1000UL

// ============================================================================
// PID
// ============================================================================
#define PID_KP            2.0f
#define PID_KI            0.02f
#define PID_KD            0.5f
#define PID_LIMIT         60.0f
#define PID_I_LIMIT       30.0f

// ============================================================================
// SOLENOID
// ============================================================================
#define SOLENOID_PULSE_MS 2000UL

// ============================================================================
// SERVO
// ============================================================================
#define DEPLOY_STOWED     0
#define DEPLOY_EXTENDED   180
#define DEPLOY_MOVE_MS    3000UL

// ============================================================================
// TELEMETRY
// ============================================================================
#define TELEM_INTERVAL_MS  2000UL
#define STANDBY_TELEM_MS   5000UL

struct __attribute__((packed)) TelemetryPacket {
    uint8_t  state;
    uint16_t missionTimeSec;
    int16_t  battMv;
    int16_t  accelZ;      // 0.01G units
    int16_t  heading;     // 0.1 degree units
    uint8_t  flags;       // bit0=lowBatt, bit1=stall
};

#endif
