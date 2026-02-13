#include "main.h"
#include "motor.h"
#include"gyro.h"

extern Motor buggy;
extern IMU_Handle_t imu_handle;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;

void delay_us(uint16_t us) {
	htim6.Instance->CNT = 0;
	HAL_TIM_Base_Start(&htim6);
	while (htim6.Instance->CNT < us);
	HAL_TIM_Base_Stop(&htim6);
}
void sendTrigger() {
	HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
}

void AutoConf(Motor *Auto) {
	Auto->motorA = &htim2;
	Auto->motorB = &htim3;
	Auto->leftMotorDir = FORWARD;
	Auto->rightMotorDir = FORWARD;
	Auto->leftMotorSpeed = BASE_SPEED;
	Auto->rightMotorSpeed = BASE_SPEED;
}
void AutoInit(Motor *Auto) {
	Brake(&buggy);
	SetMotorDirection(Auto->leftMotorDir, Auto->rightMotorDir);
	SetMotorSpeed(Auto, 220, 220);
	DirectionInit();
}
float angular_difference(float current, float target) {
	float diff = target - current;
	if (diff > 180.0f)
		diff -= 360.0f;
	if (diff < -180.0f)
		diff += 360.0f;
	return diff;
}
void DriveStraight(uint32_t duration_ms) {
	Release(&buggy);
	Forward(&buggy);
	uint32_t start = HAL_GetTick();
	while (HAL_GetTick() - start < duration_ms) {
		UpdateDirection(&buggy);
		HAL_Delay(50);
	}
	Brake(&buggy);
	//Release(&buggy);
}
void TurnRight90() {
	float initialYaw = bno055_getHeading(&imu_handle);
	float targetYaw = initialYaw + 90.0f;
	if (targetYaw >= 360.0f)
		targetYaw -= 360.0f;
	Release(&buggy);
	ForwardRight(&buggy);
	while (fabs(angular_difference(bno055_getHeading(&imu_handle), targetYaw))
			> 1.0f) {
		HAL_Delay(5);
	}
	Brake(&buggy);
	//HAL_Delay(200);
	//Release(&buggy);
}
void TurnLeft90() {
	float initialYaw = bno055_getHeading(&imu_handle);
	float targetYaw = initialYaw - 90.0f;
	if (targetYaw <= 0.f)
		targetYaw += 360.0f;
	Release(&buggy);
	ForwardLeft(&buggy);
	while (fabs(angular_difference(bno055_getHeading(&imu_handle), targetYaw))
			> 1.0f) {
		HAL_Delay(5);
	}
	Brake(&buggy);
}
void DriveRectangle() {
	for (int i = 0; i < 4; i++) {
		DriveStraight(3000);
		TurnRight90();
		DirectionInit();
	}
}
void imuInit(IMU_Handle_t * imu_handle){
		imu_handle->i2c_connect.i2c_handle = &hi2c1;
		imu_handle->i2c_connect.i2c_address = BNO055_ADDR;
		imu_handle->activation_pin.GPIOx = BNO055_Activation_GPIO_Port;
		imu_handle->activation_pin.GPIO_Pin = BNO055_Activation_Pin;

		imu_handle->imu_config.mode = BNO055_OPERATION_MODE_IMU;
		imu_handle->imu_config.eulerUnit = EULER_UNIT_DEGREES;
		imu_handle->imu_config.tempUnit = TEMP_UNIT_CELSIUS;
}
