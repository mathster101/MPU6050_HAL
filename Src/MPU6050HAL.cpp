/*
 * MPU6050HAL.cpp
 *
 *  Created on: Sep 24, 2021
 *      Author: mpanicker
 */

#include "MPU6050HAL.h"

MPU6050_HAL::MPU6050_HAL(I2C_HandleTypeDef *i2c_object, GPIO_TypeDef *sclPort,
		uint16_t sclPin, GPIO_TypeDef *sdaPort, uint16_t sdaPin) {
	i2c_handle = i2c_object;
	this->sclPort = sclPort;
	this->sclPin = sclPin;
	this->sdaPort = sdaPort;
	this->sdaPin = sdaPin;

}

/*
 * Write byte to a register
 */
HAL_StatusTypeDef MPU6050_HAL::i2c_write_byte(uint8_t addr, uint8_t data_byte) {
	HAL_StatusTypeDef result;
	uint8_t *ptr;
	ptr = &data_byte;
	result = HAL_I2C_Mem_Write(i2c_handle, DEVICE_ADDR << 1, addr,
	I2C_MEMADD_SIZE_8BIT, ptr, 1, //I2C_MEMADD_SIZE_8BIT must be used!!
			HAL_MAX_DELAY);
	if (result == HAL_BUSY) {
		i2c_busy_resolve();
		HAL_Delay(10);
		result = HAL_I2C_Mem_Write(i2c_handle, DEVICE_ADDR << 1, addr,
		I2C_MEMADD_SIZE_8BIT, ptr, 1, //I2C_MEMADD_SIZE_8BIT must be used!!
				HAL_MAX_DELAY);
	}
	return result;
}

/*
 * Read one byte from a register
 */
uint8_t MPU6050_HAL::i2c_read_byte(uint8_t addr) {
	uint8_t buffer;
	HAL_I2C_Mem_Read(i2c_handle, DEVICE_ADDR << 1, addr,
	I2C_MEMADD_SIZE_8BIT, &buffer, 1,
	HAL_MAX_DELAY);
	return buffer;
}

/*
 * Read multiple bytes from a starting register
 */
HAL_StatusTypeDef MPU6050_HAL::i2c_read_bytes(uint8_t addr, uint8_t *buffer,
		int num_bytes) {
	HAL_StatusTypeDef result;
	result = HAL_I2C_Mem_Read(i2c_handle, DEVICE_ADDR << 1, addr,
	I2C_MEMADD_SIZE_8BIT, buffer, num_bytes, HAL_MAX_DELAY);
	return result;
}

/*
 * Sensor Initialization
 */
HAL_StatusTypeDef MPU6050_HAL::initialize() {
	uint8_t reg;
	double gyro_cali[3], gx_trim_temp = 0, gy_trim_temp = 0, gz_trim_temp = 0;
	set_ranges(1, 0);
	if (i2c_write_byte(PWR_MGMT_1, 0x00) != HAL_OK)
		return HAL_ERROR;
	reg = i2c_read_byte(GYRO_CONFIG);
	reg &= 0b00000111;
	reg |= 0b00000000; // look into this
	if (i2c_write_byte(GYRO_CONFIG, reg) != HAL_OK)
		return HAL_ERROR;
	reg = i2c_read_byte(ACCEL_CONFIG);
	reg &= 0b00000111;
	reg = 0b00000000;
	if (i2c_write_byte(ACCEL_CONFIG, reg) != HAL_OK)
		return HAL_ERROR;

	int avg = 2000;
	for (int i = 0; i < avg; i++) {
		get_gyro(gyro_cali);
		gx_trim_temp += gyro_cali[0];
		gy_trim_temp += gyro_cali[1];
		gz_trim_temp += gyro_cali[2];
	}
	gx_trim = gx_trim_temp / avg;
	gy_trim = gy_trim_temp / avg;
	gz_trim = gz_trim_temp / avg;

	return HAL_OK;
}

/*
 * Returns Chip ID
 */
uint8_t MPU6050_HAL::whoami() {
	return i2c_read_byte(WHO_AM_I);
}

/*
 * Set Gyro and Accelerometer full-scale ranges
 */
HAL_StatusTypeDef MPU6050_HAL::set_ranges(int acc, int gyro) {

	/*
	 * accel_range = 2g, 4g, 8g, 16g
	 * gyro_range = 250deg/s, 500deg/s,
	 * 				1000deg/s, 2000deg/s
	 * options = 0, 1, 2 , 3
	 */
	accel_range = acc;
	gyro_range = gyro;

	switch (accel_range) {
	case 0:
		acc_scale_factor = 16384;
	case 1:
		acc_scale_factor = 8192;
	case 2:
		acc_scale_factor = 4096;
	case 3:
		acc_scale_factor = 2048;
	default:
		acc_scale_factor = 16384;
	}

	switch (gyro_range) {
	case 0:
		gyro_scale_factor = 131;
	case 1:
		gyro_scale_factor = 65.5;
	case 2:
		gyro_scale_factor = 32.8;
	case 3:
		gyro_scale_factor = 16.4;
	default:
		gyro_scale_factor = 131;
	}
	return HAL_OK;
}

/*
 * Read Accelerometer data
 */
HAL_StatusTypeDef MPU6050_HAL::get_accel(double *acc_buf) {
	uint8_t buffer[6];
	int16_t ax_raw, ay_raw, az_raw;
	if (i2c_read_bytes(ACCEL_REG_START, (uint8_t*) buffer, 6) != HAL_OK)
		return HAL_ERROR;
	ax_raw = (buffer[0] << 8 | buffer[1]);
	ay_raw = (buffer[2] << 8 | buffer[3]);
	az_raw = (buffer[4] << 8 | buffer[5]);
	acc_buf[0] = (double) ax_raw / acc_scale_factor;
	acc_buf[1] = (double) ay_raw / acc_scale_factor;
	acc_buf[2] = (double) az_raw / acc_scale_factor;
	ax_internal = acc_buf[0];
	ay_internal = acc_buf[1];
	az_internal = acc_buf[2];
	return HAL_OK;
}

/*
 * Read Gyro data
 */
HAL_StatusTypeDef MPU6050_HAL::get_gyro(double *gyro_buf) {
	uint8_t buffer[6];
	int16_t gx_raw, gy_raw, gz_raw;
	if (i2c_read_bytes(GYRO_REG_START, (uint8_t*) buffer, 6) != HAL_OK)
		return HAL_ERROR;
	gx_raw = (buffer[0] << 8 | buffer[1]);
	gy_raw = (buffer[2] << 8 | buffer[3]);
	gz_raw = (buffer[4] << 8 | buffer[5]);
	gyro_buf[0] = (double) gx_raw / gyro_scale_factor - gx_trim;
	gyro_buf[1] = (double) gy_raw / gyro_scale_factor - gy_trim;
	gyro_buf[2] = (double) gz_raw / gyro_scale_factor - gz_trim;
	return HAL_OK;
}

/*
 * Returns absolute value of acceleration in Gs
 */
double MPU6050_HAL::accel_magnitude() {
	double accel[3], temp;
	if (get_accel(accel) != HAL_OK)
		return -1;
	temp = pow(accel[0], 2) + pow(accel[1], 2) + pow(accel[2], 2);
	return sqrt(temp);
}

/*
 * Calculates roll and pitch using the accelerometer (degrees)
 */
//pitch = 180 * atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ))/PI;
//roll = 180 * atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ))/PI;
HAL_StatusTypeDef MPU6050_HAL::get_rp_acc(double *angle_buf) {
	double acc_buffer[3]; // ax, ay, az
	get_accel(acc_buffer);
	angle_buf[0] = 180
			* atan2(acc_buffer[1],
					sqrt(
							acc_buffer[0] * acc_buffer[0]
									+ acc_buffer[2] * acc_buffer[2])) / M_PI;
	angle_buf[1] = 180
			* atan2(acc_buffer[0],
					sqrt(
							acc_buffer[1] * acc_buffer[1]
									+ acc_buffer[2] * acc_buffer[2])) / M_PI;
	return HAL_OK;
}

/*
 * Calculates roll and pitch using the gyro (degrees)
 */
HAL_StatusTypeDef MPU6050_HAL::get_rp_gyr(double *angle_buf) {
	if (!gyro_first_call) { // on first gyro call, get values from accel
		get_rp_acc(angle_buf);
		angle_buf[2] = 0;
		sys_tick = HAL_GetTick();
		gx_internal = angle_buf[0];
		gy_internal = angle_buf[1];
		gz_internal = angle_buf[2];
		gyro_first_call = true;
	} else {
		double gyro_buf[3];
		uint32_t new_tick = HAL_GetTick();
		elapsed = (int32_t) (new_tick - sys_tick);
		elapsed /= 1000;
		get_gyro(gyro_buf);
		angle_buf[0] = gx_internal + gyro_buf[0] * (elapsed);
		angle_buf[1] = gy_internal + gyro_buf[1] * (elapsed);
		angle_buf[2] = gz_internal + gyro_buf[2] * (elapsed);
		gx_internal = angle_buf[0];
		gy_internal = angle_buf[1];
		gz_internal = angle_buf[2];
		sys_tick = new_tick;
	}
	return HAL_OK;
}

void MPU6050_HAL::i2c_busy_resolve() {
	GPIO_InitTypeDef GPIO_InitStructure;

	// 1. Clear PE bit.
	i2c_handle->Instance->CR1 &= ~(0x0001);

	//  2. Configure the SCL and SDA I/Os as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStructure.Alternate = GPIO_AF4_I2C1;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStructure.Pin = sclPin;
	HAL_GPIO_Init(sclPort, &GPIO_InitStructure);
	HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_SET);

	GPIO_InitStructure.Pin = sdaPin;
	HAL_GPIO_Init(sdaPort, &GPIO_InitStructure);
	HAL_GPIO_WritePin(sdaPort, sdaPin, GPIO_PIN_SET);

	// 3. Check SCL and SDA High level in GPIOx_IDR.
	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(sclPort, sclPin)) {
		asm("nop");
	}

	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(sdaPort, sdaPin)) {
		asm("nop");
	}

	// 4. Configure the SDA I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	HAL_GPIO_WritePin(sdaPort, sdaPin, GPIO_PIN_RESET);

	//  5. Check SDA Low level in GPIOx_IDR.
	while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(sdaPort, sdaPin)) {
		asm("nop");
	}

	// 6. Configure the SCL I/O as General Purpose Output Open-Drain, Low level (Write 0 to GPIOx_ODR).
	HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_RESET);

	//  7. Check SCL Low level in GPIOx_IDR.
	while (GPIO_PIN_RESET != HAL_GPIO_ReadPin(sclPort, sclPin)) {
		asm("nop");
	}

	// 8. Configure the SCL I/O as General Purpose Output Open-Drain, High level (Write 1 to GPIOx_ODR).
	HAL_GPIO_WritePin(sclPort, sclPin, GPIO_PIN_SET);

	// 9. Check SCL High level in GPIOx_IDR.
	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(sclPort, sclPin)) {
		asm("nop");
	}

	// 10. Configure the SDA I/O as General Purpose Output Open-Drain , High level (Write 1 to GPIOx_ODR).
	HAL_GPIO_WritePin(sdaPort, sdaPin, GPIO_PIN_SET);

	// 11. Check SDA High level in GPIOx_IDR.
	while (GPIO_PIN_SET != HAL_GPIO_ReadPin(sdaPort, sdaPin)) {
		asm("nop");
	}

	// 12. Configure the SCL and SDA I/Os as Alternate function Open-Drain.
	GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
	//GPIO_InitStructure.Alternate = I2C_PIN_MAP;

	GPIO_InitStructure.Pin = sclPin;
	HAL_GPIO_Init(sclPort, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = sdaPin;
	HAL_GPIO_Init(sdaPort, &GPIO_InitStructure);

	// 13. Set SWRST bit in I2Cx_CR1 register.
	i2c_handle->Instance->CR1 |= 0x8000;

	asm("nop");

	// 14. Clear SWRST bit in I2Cx_CR1 register.
	i2c_handle->Instance->CR1 &= ~0x8000;

	asm("nop");

	// 15. Enable the I2C peripheral by setting the PE bit in I2Cx_CR1 register
	i2c_handle->Instance->CR1 |= 0x0001;

	// Call initialization function.
	HAL_I2C_Init((i2c_handle));
}
