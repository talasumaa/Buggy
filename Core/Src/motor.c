/*
 * motor.c
 *
 *  Created on: Jun 13, 2025
 *      Author: User
 */
#include "motor.h"
#include "gyro.h"
#include "main.h"

//static Motor Auto;

#define CorrectionRatio 0.8f // Proportionalfaktor für einfache Regelung

//static uint16_t yaw_offset = 0;

void SetMotorDirection(uint8_t motorA_forward, uint8_t motorB_forward){
	//	rechter Motor
	HAL_GPIO_WritePin(IN1_Port, IN1_Pin, motorA_forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
//	HAL_GPIO_WritePin(IN2_Port, IN2_Pin, motorA_forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
	//	linker Motor
	HAL_GPIO_WritePin(IN3_Port, IN3_Pin, motorB_forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(IN4_Port, IN4_Pin, motorB_forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void SetMotorBrakes(uint8_t motorA, uint8_t motorB){
	HAL_GPIO_WritePin(IN2_Port, IN2_Pin, motorA ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_Port, IN4_Pin, motorB ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void SetMotorSpeed(Motor* m, uint8_t speedA, uint8_t speedB){
	 if (speedA > 255) speedA = 255;
	 if (speedB> 255) speedB = 255;
	 m->motorA->Instance->CCR2 = speedA;
	 m->motorB->Instance->CCR2 = speedB;
}
void DirectionInit(void){
	yaw_offset = IMU_GetYaw();
}

void UpdateDirection(Motor* m){
    float currentYaw = IMU_GetYaw();

    // Calculate wrapped error (-180° to 180°)
    float error = currentYaw - yaw_offset;
    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    // Debug output (enable temporarily)
    sprintf(message, "Yaw: %.1f Err: %.1f\r\n", currentYaw, error);
    HAL_UART_Transmit(&huart2, message, strlen(message), 100);

    float correction = CorrectionRatio * error;

    float speedA = BASE_SPEED + correction;  // Left motor
    float speedB = BASE_SPEED - correction;  // Right motor

    // Clamp speeds
    speedA = (speedA < 0) ? 0 : (speedA > 255) ? 255 : speedA;
    speedB = (speedB < 0) ? 0 : (speedB > 255) ? 255 : speedB;

    SetMotorSpeed(m, (uint8_t)speedA, (uint8_t)speedB);
}

void Backward(Motor* m){
	SetMotorDirection(BACKWARD, BACKWARD);
	SetMotorSpeed(m, BASE_SPEED, BASE_SPEED);
}

void BackwardLeft(Motor* m){
	SetMotorDirection(BACKWARD, BACKWARD);
	SetMotorSpeed(m, 0, BASE_SPEED);
}

void BackwardRight(Motor* m){
	SetMotorDirection(BACKWARD, BACKWARD);
	SetMotorSpeed(m, BASE_SPEED, 0);
}

void Forward(Motor* m){
	SetMotorDirection(FORWARD, FORWARD);
	SetMotorSpeed(m, BASE_SPEED, BASE_SPEED);
}

void ForwardLeft(Motor* m){
	SetMotorDirection(FORWARD, FORWARD);
	SetMotorSpeed(m, BASE_SPEED, 0);
}

void ForwardRight(Motor* m){
	SetMotorDirection(FORWARD, FORWARD);
	SetMotorSpeed(m, 0, BASE_SPEED);
}

void Brake(Motor* m){
	SetMotorBrakes(BRAKE_ENGAGED, BRAKE_ENGAGED);
}

void Release(Motor* m){
	SetMotorBrakes(BRAKE_RELEASED, BRAKE_RELEASED);
}

//TIM_HandleTypeDef* getMotor(Motor* m, uint8_t left_right){
//	if(left_right == 0){
//		return m->motorA;
//	}
//	else {
//		return m->motorB;
//	}
//}



