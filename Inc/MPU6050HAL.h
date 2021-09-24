/*
 * MPU6050HAL.h
 *
 *  Created on: Sep 24, 2021
 *      Author: mpanicker
 */

#ifndef SRC_MPU6050HAL_H_
#define SRC_MPU6050HAL_H_

#include <stm32f4xx_hal.h>
#include <math.h>

//DEFINES
#define DEVICE_ADDR 0x68
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_REG_START 0x3B
#define TEMP_REG_START 0x41
#define GYRO_REG_START 0x43
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define WHO_AM_I 0x75

class MPU6050_HAL {
public:

	MPU6050_HAL(I2C_HandleTypeDef *i2c_object);
	HAL_StatusTypeDef initialize();
	HAL_StatusTypeDef set_ranges(int acc, int gyro);
	HAL_StatusTypeDef get_accel(double *acc_buf);
	HAL_StatusTypeDef get_gyro(double *gyro_buf);
	HAL_StatusTypeDef get_rp_acc(double *angle_buf);
	double accel_magnitude();
	uint8_t whoami();

private:
	uint8_t accel_range, gyro_range;
	float acc_scale_factor, gyro_scale_factor;
	double ax_internal, ay_internal, az_internal;
	I2C_HandleTypeDef *i2c_handle;

	HAL_StatusTypeDef i2c_write_byte(uint8_t addr, uint8_t data_byte);
	uint8_t i2c_read_byte(uint8_t addr);
	HAL_StatusTypeDef i2c_read_bytes(uint8_t addr, uint8_t *buffer,
			int num_bytes = 1);
};

#endif /* SRC_MPU6050HAL_H_ */
