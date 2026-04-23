/**
 * ============================================================================
 * PROJECT LAKSHMI — Rover HAL Header (Arduino Nano)
 * ============================================================================
 */

#ifndef ROVER_HAL_H
#define ROVER_HAL_H

#include "Hardware_Config.h"

void  halInitAll();
void  halDriveMotors(int16_t leftPWM, int16_t rightPWM);
void  halStopMotors();
void  halFireSolenoid(bool on);
float halReadBatteryVoltage();
bool  halReadAccelGs(float &ax, float &ay, float &az);
float halReadYawDeg();
void  halTransmitLoRa(const TelemetryPacket &pkt);
float halPIDCompute(float target, float current, float &integral,
                    float &prevErr, unsigned long &lastMs);
float halWrapAngle(float a);

#endif
