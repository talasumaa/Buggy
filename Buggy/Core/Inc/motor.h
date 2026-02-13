/*
 * motor.h
 *
 *  Created on: Jun 10, 2025
 *      Author: User
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>
#include "main.h"
#include "gyro.h"

extern float yaw_offset;
extern UART_HandleTypeDef huart2;
extern uint8_t message[60];


// Motor Pins
#define IN1_Pin GPIO_PIN_6		//	DIR A
#define IN2_Pin GPIO_PIN_7		//	BRAKE A
#define IN3_Pin GPIO_PIN_5		//	DIR B
#define IN4_Pin GPIO_PIN_9		//	BRAKE B

#define IN1_Port GPIOA
#define IN2_Port GPIOC
#define IN3_Port GPIOA
#define IN4_Port GPIOA

// Richtung
#define FORWARD 1
#define BACKWARD 0
#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define BRAKE_ENGAGED 1
#define BRAKE_RELEASED 0

//default speed of motors: 150 rpm
#define BASE_SPEED 220	//220 previous value

typedef struct{
	TIM_HandleTypeDef* motorA;
	TIM_HandleTypeDef* motorB;
	uint8_t leftMotorSpeed;
	uint8_t rightMotorSpeed;
	uint8_t leftMotorDir;
	uint8_t rightMotorDir;
}Motor;

TIM_HandleTypeDef* getMotor(Motor* m, uint8_t left_right);

void SetMotorDirection(uint8_t motorA_forward, uint8_t motorB_forward);
void SetMotorSpeed(Motor* m, uint8_t, uint8_t);
void SetMotorBrakes(uint8_t motorA, uint8_t motorB);
void DirectionInit(void);
void UpdateDirection(Motor* m);


void Backward(Motor* m);
void BackwardLeft(Motor* m);
void BackwardRight(Motor* m);
void Forward(Motor* m);
void ForwardLeft(Motor* m);
void ForwardRight(Motor* m);
void Brake(Motor* m);
void Release(Motor* m);


#endif /* INC_MOTOR_H_ */
