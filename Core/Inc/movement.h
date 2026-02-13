#pragma once

void AutoConf(Motor *Auto);
void imuInit(IMU_Handle_t * imu_handle);
void AutoInit(Motor *Auto);
float angular_difference(float current, float target);
void DriveStraight(uint32_t duration_ms);
void TurnRight90(void);
void TurnLeft90(void);
void DriveRectangle(void);
void sendTrigger(void);
