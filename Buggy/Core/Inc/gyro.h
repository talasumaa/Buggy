/*
 * gyro.h
 *
 *  Created on: Jun 13, 2025
 *      Author: User
 */

#ifndef INC_GYRO_H_
#define INC_GYRO_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <sys/time.h>
#include <math.h>

extern UART_HandleTypeDef huart2;
extern uint8_t message[60];

// ToDo: Ergänzen Sie die Defines mit den Werten für die Einheiten und das Reset des Sensors
#define EULER_UNIT_DEGREES			0
#define EULER_UNIT_RAD				1
#define TEMP_UNIT_CELSIUS			0
#define TEMP_UNIT_FAHRENHEIT		1
#define BNO055_RESET_SYSTEM			0x20U

#define BNO055_CHIP_ID_REG			0x00U
#define BNO055_CHIP_ID_VAL			0xA0

#define BNO055_CALIB_STAT			0x35U
#define BNO055_ADDR					0x28U

// ToDo: Ergänzen Sie die Defines mit den Registeradressen
// Register definitions
#define BNO055_UNIT_SEL				0x3BU
#define BNO055_OPR_MODE				0x3DU
#define BNO055_EUL_HEADING_LSB		0x1AU
#define BNO055_EUL_HEADING_MSB		0x1BU
#define BNO055_EUL_ROLL_LSB			0x1CU
#define BNO055_EUL_ROLL_MSB			0x1DU
#define BNO055_EUL_PITCH_LSB		0x1EU
#define BNO055_EUL_PITCH_MSB		0x1FU
#define BNO055_TEMP					0x34U
#define BNO055_SYS_TRIGGER			0x3FU
#define BNO055_TEMP_SOURCE			0x40U

#define TEMP_SOURCE_GYRO			0x01U
#define BNO055_PAGE_ID				0x07U
#define BNO055_PWR_MODE				0x3E

// ToDo: Ergänzen Sie die Defines für alle Operationsmodi
// Operating Modes

//	--	Page 20	--
#define BNO055_OPERATION_MODE_CONFIG		0x00U
//#define BNO055_OPERATION_MODE_ACCONLY		0x01U
//#define BNO055_OPERATION_MODE_MAGONLY		0x02U
//#define BNO055_OPERATION_MODE_GYROONLY	0x03U
//#define BNO055_OPERATION_MODE_ACCMAG		0x04U
//#define BNO055_OPERATION_MODE_ACCGYRO		0x05U
//#define BNO055_OPERATION_MODE_MAGGYRO		0x06U
//#define BNO055_OPERATION_MODE_AMG			0x07U
#define BNO055_OPERATION_MODE_IMU			0x08U
//#define BNO055_OPERATION_MODE_COMPASS		0x09U
//#define BNO055_OPERATION_MODE_M4G			0x0AU
//#define BNO055_OPERATION_MODE_NDOF_FMC_OFF0x0BU
#define BNO055_OPERATION_MODE_NDOF			0x0CU
// Sensor Modes

//	--	Page 26	--

//	Accelerometer
#define BNO055_ACC_G_RANGE_2G					0x00U
#define BNO055_ACC_G_RANGE_4G					0x01U
#define BNO055_ACC_G_RANGE_8G					0x10U
#define BNO055_ACC_G_RANGE_16G					0x11U

#define BNO055_ACC_BANDWIDTH_7_81_Hz			0x00U
#define BNO055_ACC_BANDWIDTH_15_63_Hz			0x04U
#define BNO055_ACC_BANDWIDTH_31_25_Hz			0x08U
#define BNO055_ACC_BANDWIDTH_62_5_Hz			0x0CU
#define BNO055_ACC_BANDWIDTH_125_Hz				0x10U
#define BNO055_ACC_BANDWIDTH_250_Hz				0x14U
#define BNO055_ACC_BANDWIDTH_500_Hz				0x18U
#define BNO055_ACC_BANDWIDTH_1000_Hz			0x1CU

#define BNO055_ACC_OPERATION_MODE_NORMAL		0x00U
#define BNO055_ACC_OPERATION_MODE_SUSPEND		0x20U
#define BNO055_ACC_OPERATION_MODE_LOW_POWER_1	0x40U
#define BNO055_ACC_OPERATION_MODE_STANDBY		0x60U
#define BNO055_ACC_OPERATION_MODE_LOW_POWER_2	0x80U
#define BNO055_ACC_OPERATION_MODE_DEEP_SUSPEND	0xA0U


//Gyroscope
#define BNO055_GYR_RANGE_2000_DPS				0x00U
#define BNO055_GYR_RANGE_1000_DPS				0x01U
#define BNO055_GYR_RANGE_500_DPS				0x02U
#define BNO055_GYR_RANGE_250_DPS				0x03U
#define BNO055_GYR_RANGE_125_DPS				0x04U

#define BNO055_GYR_BANDWIDTH_523_Hz				0x00U
#define BNO055_GYR_BANDWIDTH_230_Hz				0x08U
#define BNO055_GYR_BANDWIDTH_116_Hz				0x10U
#define BNO055_GYR_BANDWIDTH_47_Hz				0x18U
#define BNO055_GYR_BANDWIDTH_23_Hz				0x20U
#define BNO055_GYR_BANDWIDTH_12_Hz				0x28U
#define BNO055_GYR_BANDWIDTH_64_Hz				0x30U
#define BNO055_GYR_BANDWIDTH_32_Hz				0x38U

#define BNO055_GYR_OPERATION_MODE_NORMAL				0x00U
#define BNO055_GYR_OPERATION_MODE_FAST_POWER_UP			0x01U
#define BNO055_GYR_OPERATION_MODE_DEEP_SUSPEND			0x02U
#define BNO055_GYR_OPERATION_MODE_SUSPEND				0x03U
#define BNO055_GYR_OPERATION_MODE_ADVANCED_POWERSAVE	0x04U

//Magnetometer

#define BNO055_MAG_DATA_OUTPUT_RATE_2_Hz		0x00U
#define BNO055_MAG_DATA_OUTPUT_RATE_6_Hz		0x01U
#define BNO055_MAG_DATA_OUTPUT_RATE_8_Hz		0x02U
#define BNO055_MAG_DATA_OUTPUT_RATE_10_Hz		0x03U
#define BNO055_MAG_DATA_OUTPUT_RATE_15_Hz		0x04U
#define BNO055_MAG_DATA_OUTPUT_RATE_20_Hz		0x05U
#define BNO055_MAG_DATA_OUTPUT_RATE_25_Hz		0x06U
#define BNO055_MAG_DATA_OUTPUT_RATE_30_Hz		0x07U

#define BNO055_MAG_OPERATION_MODE_LOW_POWER			0x00U
#define BNO055_MAG_OPERATION_MODE_REGULAR			0x08U
#define BNO055_MAG_OPERATION_MODE_ENHANCED_REGULAR	0x10U
#define BNO055_MAG_OPERATION_MODE_HIGH_ACCURACY		0x18U

#define BNO055_MAG_POWER_MODE_NORMAL				0x00U
#define BNO055_MAG_POWER_MODE_SLEEP					0x20U
#define BNO055_MAG_POWER_MODE_SUSPEND				0x40U
#define BNO055_MAG_POWER_MODE_FORCE_MODE			0x60U

// Fusion Modes





/*
 * Handle structs for sensor interaction
 */
// ToDo: Ergänzen Sie das struct, so dass es
//			- den Modus -- welchen Modus?
//			- die Einheit der Eulerwinkel
// 			- die Einheit der Temperatur enthält
typedef struct
{
	uint8_t mode;		//	NDOF / IMU p 20
	uint8_t eulerUnit;	//0 is degrees, 1 is rad
	uint8_t tempUnit;	//0 is celsius, 1 is fahrenheit

}IMU_Config_t;

typedef struct
{
	I2C_HandleTypeDef *i2c_handle;
	uint8_t i2c_address;
}I2C_Handle_t;

typedef struct
{
	GPIO_TypeDef *GPIOx;
	uint32_t GPIO_Pin;
}GPIO_Handle_t;

typedef struct
{
	I2C_Handle_t i2c_connect;
	GPIO_Handle_t activation_pin;
	IMU_Config_t imu_config;
}IMU_Handle_t;


void bno055_init(IMU_Handle_t *imu_handle);
void bno055_reset(IMU_Handle_t *imu_handle);
void bno055_deinit(IMU_Handle_t *imu_handle);

uint8_t bno055_getOperationMode(IMU_Handle_t *imu_handle);
void bno055_setOperationMode(IMU_Handle_t *imu_handle, uint8_t mode);

float bno055_getHeading(IMU_Handle_t *imu_handle);
int16_t bno055_getTemp(IMU_Handle_t *imu_handle);
uint8_t getCal(IMU_Handle_t *imu_handle);
HAL_StatusTypeDef bno055_writeData(I2C_Handle_t *i2c_handle, uint8_t reg, uint8_t data);
HAL_StatusTypeDef bno055_readData(I2C_Handle_t *i2c_handle, uint8_t reg, uint8_t *data, uint8_t len);

float IMU_GetYaw();


#endif /* INC_GYRO_H_ */
