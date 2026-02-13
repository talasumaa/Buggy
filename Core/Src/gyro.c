/*
 * gyro.c
 *
 *  Created on: Jun 13, 2025
 *      Author: User
 */
#include "gyro.h"
extern IMU_Handle_t imu_handle;

void bno055_reset(IMU_Handle_t *imu_handle) {
	bno055_writeData(&(imu_handle->i2c_connect), BNO055_SYS_TRIGGER, BNO055_RESET_SYSTEM);
	HAL_Delay(700);
}

void bno055_init(IMU_Handle_t *imu_handle){

	//Set activation PIN to high
	GPIO_TypeDef* activation_port 	= imu_handle->activation_pin.GPIOx;
	uint32_t activation_pin 		= imu_handle->activation_pin.GPIO_Pin;
	HAL_GPIO_WritePin(activation_port, activation_pin, GPIO_PIN_SET);

	//Reset in the beginning
	bno055_reset(imu_handle);
	HAL_Delay(700);

	//Set register page to 0
	bno055_writeData(&imu_handle->i2c_connect, BNO055_PAGE_ID, 0x00);
	HAL_Delay(10);

	//Write PWR_MODE normal
	bno055_writeData(&imu_handle->i2c_connect, BNO055_PWR_MODE, 0x00);
	HAL_Delay(10);

	//Dont know what it means ..
	bno055_writeData(&(imu_handle->i2c_connect), BNO055_SYS_TRIGGER, 0x0);
	bno055_setOperationMode(imu_handle, BNO055_OPERATION_MODE_CONFIG);

	HAL_Delay(20);


	//Set temperature and euler unit
	uint8_t unit_sel_value = 0x00;
	uint8_t eulerEinheit = imu_handle->imu_config.eulerUnit;
	uint8_t tempEinheit = imu_handle->imu_config.tempUnit;
	unit_sel_value = (tempEinheit << 4) | (eulerEinheit << 2);

	bno055_writeData(&(imu_handle->i2c_connect), BNO055_UNIT_SEL, unit_sel_value);
	HAL_Delay(20);

	//Set temperature source
	bno055_writeData(&(imu_handle->i2c_connect), BNO055_TEMP_SOURCE, TEMP_SOURCE_GYRO);
	HAL_Delay(20);

	//Set operation mode
	bno055_setOperationMode(imu_handle, imu_handle->imu_config.mode);
	HAL_Delay(50);

	// Verify mode switch
	uint8_t mode = bno055_getOperationMode(imu_handle);
	sprintf(message, "Current mode: 0x%02X\r\n", mode);
	HAL_UART_Transmit(&huart2, message, strlen(message), 100);
}


void bno055_deinit(IMU_Handle_t *imu_handle){
	bno055_reset(imu_handle);

	HAL_GPIO_WritePin(imu_handle->activation_pin.GPIOx, imu_handle->activation_pin.GPIO_Pin, GPIO_PIN_RESET);
}

uint8_t bno055_getOperationMode(IMU_Handle_t *imu_handle){
	uint8_t data = 0;
	bno055_readData(&(imu_handle->i2c_connect), BNO055_OPR_MODE, &data, 1);
	return  data;
}

void bno055_setOperationMode(IMU_Handle_t *imu_handle, uint8_t mode){
	bno055_writeData(&(imu_handle->i2c_connect), BNO055_OPR_MODE, mode);
	HAL_Delay(20);
}

float bno055_getHeading(IMU_Handle_t *imu_handle) {
    uint8_t data[2] = {0, 0};
    int16_t raw_heading = 0;

    bno055_readData(&imu_handle->i2c_connect, BNO055_EUL_HEADING_LSB, data, 2);

    raw_heading = (int16_t)((data[1] << 8) | data[0]);

    if(imu_handle->imu_config.eulerUnit == EULER_UNIT_DEGREES) {
        return raw_heading / 16.0f;  // 1° = 16 LSB
    } else {
        return raw_heading / 900.0f; // 1 rad = 900 LSB
    }
}

int16_t bno055_getTemp(IMU_Handle_t *imu_handle){
	uint8_t temp = 0;
	int16_t output = 0;
 	bno055_readData(&(imu_handle->i2c_connect), 0x34, &temp, 1);
	if(imu_handle->imu_config.tempUnit == TEMP_UNIT_CELSIUS)
		output = (int16_t)temp;	//	1 °C = 1 LSB
	else if(imu_handle->imu_config.tempUnit == TEMP_UNIT_FAHRENHEIT)
		output = (int16_t)temp*2;	// 2 F = 1 LSB
	return output;
}

uint8_t getCal(IMU_Handle_t *imu_handle){
	uint8_t cal = 0;
	bno055_readData(&(imu_handle->i2c_connect), BNO055_CALIB_STAT, &cal, 1);
	return cal;
}

HAL_StatusTypeDef bno055_writeData(I2C_Handle_t *i2c_handle, uint8_t reg, uint8_t data) {
	uint8_t txdata[2] = {reg, data};
	uint8_t status;
	status = HAL_I2C_Master_Transmit(i2c_handle->i2c_handle, i2c_handle->i2c_address << 1, txdata, sizeof(txdata), 10);
	return status;
}

HAL_StatusTypeDef bno055_readData(I2C_Handle_t *i2c_handle, uint8_t reg, uint8_t *data, uint8_t len) {
	uint8_t status;
	status = HAL_I2C_Master_Transmit(i2c_handle->i2c_handle, i2c_handle->i2c_address << 1, &reg, 1, 100);
	if (status == HAL_OK){
		status = HAL_I2C_Master_Receive(i2c_handle->i2c_handle, i2c_handle->i2c_address << 1, data, len, 100);
	}
	return status;
}

float IMU_GetYaw(){
	return bno055_getHeading(&imu_handle);
}

