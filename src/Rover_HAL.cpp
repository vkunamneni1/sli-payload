/**
 * ============================================================================
 * PROJECT LAKSHMI — HAL Implementation (Arduino Nano)
 * ============================================================================
 * Deploy + traverse only. No soil collection hardware.
 * 
 * MOTOR CONTROL: DIR + PWM convention.
 *   DIR LOW  + analogWrite = forward
 *   DIR HIGH + analogWrite = reverse
 * 
 * IMU: MPU6050 via raw I2C (saves ~4KB flash vs library).
 *   Accel at +/-16G range (LSB = 2048/G) for launch detection up to 15G.
 *   Gyro Z integrated for relative heading (drifts, but fine for 8s traverse).
 * ============================================================================
 */

#include "Rover_HAL.h"
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>

// ============================================================================
// INIT
// ============================================================================

void halInitAll() {
    // Motors
    pinMode(PIN_LEFT_PWM, OUTPUT);
    pinMode(PIN_LEFT_DIR, OUTPUT);
    pinMode(PIN_RIGHT_PWM, OUTPUT);
    pinMode(PIN_RIGHT_DIR, OUTPUT);
    halStopMotors();

    // Solenoid
    pinMode(PIN_SOLENOID, OUTPUT);
    digitalWrite(PIN_SOLENOID, LOW);

    // I2C + MPU6050
    Wire.begin();
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);  // wake up (clear sleep bit)
    Wire.endTransmission();
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x1C);
    Wire.write(0x18);  // +/-16G accel range
    Wire.endTransmission();

    // LoRa pins (actual SPI init happens in LoRa.begin())
    LoRa.setPins(PIN_LORA_CS, PIN_LORA_RST, PIN_LORA_DIO0);
}

// ============================================================================
// MOTORS
// ============================================================================

void halDriveMotors(int16_t leftPWM, int16_t rightPWM) {
    leftPWM  = constrain(leftPWM, -255, 255);
    rightPWM = constrain(rightPWM, -255, 255);

    if (leftPWM >= 0) {
        digitalWrite(PIN_LEFT_DIR, LOW);
        analogWrite(PIN_LEFT_PWM, (uint8_t)leftPWM);
    } else {
        digitalWrite(PIN_LEFT_DIR, HIGH);
        analogWrite(PIN_LEFT_PWM, (uint8_t)(-leftPWM));
    }

    if (rightPWM >= 0) {
        digitalWrite(PIN_RIGHT_DIR, LOW);
        analogWrite(PIN_RIGHT_PWM, (uint8_t)rightPWM);
    } else {
        digitalWrite(PIN_RIGHT_DIR, HIGH);
        analogWrite(PIN_RIGHT_PWM, (uint8_t)(-rightPWM));
    }
}

void halStopMotors() {
    analogWrite(PIN_LEFT_PWM, 0);
    analogWrite(PIN_RIGHT_PWM, 0);
}

// ============================================================================
// SOLENOID
// ============================================================================

void halFireSolenoid(bool on) {
    digitalWrite(PIN_SOLENOID, on ? HIGH : LOW);
}

// ============================================================================
// BATTERY
// ============================================================================

float halReadBatteryVoltage() {
    uint16_t raw = analogRead(PIN_BATT);
    float vAdc = (float)raw * (5.0f / 1023.0f);
    return vAdc / VDIV_RATIO;
}

// ============================================================================
// IMU — MPU6050 raw I2C
// ============================================================================

static int16_t mpuRead16(uint8_t reg) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)2);
    int16_t hi = Wire.read();
    int16_t lo = Wire.read();
    return (hi << 8) | lo;
}

bool halReadAccelGs(float &ax, float &ay, float &az) {
    // +/-16G range: 1G = 2048 LSB
    ax = (float)mpuRead16(0x3B) / 2048.0f;
    ay = (float)mpuRead16(0x3D) / 2048.0f;
    az = (float)mpuRead16(0x3F) / 2048.0f;
    return true;
}

// Gyro-integrated yaw (relative heading)
static float yawAccum = 0.0f;
static unsigned long lastGyroMs = 0;

float halReadYawDeg() {
    unsigned long now = millis();
    if (lastGyroMs == 0) {
        lastGyroMs = now;
        return 0.0f;
    }

    float dt = (float)(now - lastGyroMs) / 1000.0f;
    lastGyroMs = now;

    // +/-250 deg/s default range: 1 deg/s = 131 LSB
    float dps = (float)mpuRead16(0x47) / 131.0f;
    yawAccum += dps * dt;
    return yawAccum;
}

// ============================================================================
// LoRa
// ============================================================================

void halTransmitLoRa(const TelemetryPacket &pkt) {
    LoRa.beginPacket();
    LoRa.write((const uint8_t *)&pkt, sizeof(TelemetryPacket));
    LoRa.endPacket(true);  // true = async (non-blocking)
}

// ============================================================================
// PID
// ============================================================================

float halPIDCompute(float target, float current, float &integral,
                    float &prevErr, unsigned long &lastMs) {
    unsigned long now = millis();
    float dt = (float)(now - lastMs) / 1000.0f;
    if (dt <= 0.0f) return 0.0f;
    lastMs = now;

    float error = halWrapAngle(target - current);

    // P
    float p = PID_KP * error;
    // I with anti-windup
    integral += error * dt;
    integral = constrain(integral, -PID_I_LIMIT, PID_I_LIMIT);
    float i = PID_KI * integral;
    // D
    float d = PID_KD * (error - prevErr) / dt;
    prevErr = error;

    float out = p + i + d;
    return constrain(out, -PID_LIMIT, PID_LIMIT);
}

float halWrapAngle(float a) {
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}
