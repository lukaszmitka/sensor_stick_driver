//
// Created by lukasz on 17.06.16.
//

#ifndef SENSOR_STICK_DRIVER_MAIN_H
#define SENSOR_STICK_DRIVER_MAIN_H

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "gnublin.h"

#define BOARD RASPBERRY_PI

// accelerometer constants
#define ADXL345_ADDRESS 0x53
#define ADXL_REGISTER_PWRCTL 0x2D
#define ADXL_REGISTER_BW_RATE 0x2C
#define ADXL_DATA_FORMAT 0x31
#define ADXL_PWRCTL_MEASURE 0x8
#define ADXL_SCALE_FACTOR 0.0039f // 3.9 mg (if range is set to +-2g)

// gyro constants
#define ITG_3200_ADDRESS 0x68 // device address
#define ITG_3200_SENS_REG 0x1B // 8 bytes: T_H, T_L, GX_H, GX_L, GY_H, GY_L, GZ_H, GZ_L
#define ITG_3200_WHO_AM_I 0x00 // identification register
#define ITG_3200_POWER_REG 0x3E
#define ITG_3200_POWER_DATA 0b00000011 // reset, sleep stby off; PLL with Z Gyro reference
#define ITG_3200_DLPF_FS_REG 0x16
#define ITG_3200_DLPF_FS_DATA 0b00011100 // FullScale +-2000deg/s; DIgital Low Pass Filter: 20Hz
#define ITG_3200_SR_DIV_REG 0x15
#define ITG_3200_SR_DIV_DATA 0b00000100 // 200Hz output
#define ITG_3200_SCALE_FACTOR 14.375f

// magnetometer constants
#define HMC5883L_ADDRESS 0x1e // device address
#define HMC5883L_CONF_REG_A 0x00 // configuration register A
#define HMC5883L_CONF_REG_B 0x01 // configuration register B
#define HMC5883L_MODE_REG 0x02 // mode register
#define HMC5883L_SENS_REG 0x03 // 6 bytes X_H, X_L, Z_H, Z_L, Y_H, Y_L
#define HMC5883L_STATUS_REG 0x09 // status register
#define HMC5883L_CONF_DATA_A 0b01111000 // 8sample average, 75Hz, normal measurement mode
#define HMC5883L_CONF_DATA_B 0b00100000 // range +-1.3Ga, 0,92mGa/LSb
#define HMC5883L_SCALE 0.00092f
#define HMC5883L_MODE_DATA 0b00000000 // continous measurement mode


// unit constants
#define g_to_m_s 9.8f // 1g = 9.8 m/s^2
#define T_to_Gs 10000 // 1 Tesla = 10000 Gaus
#define Gs_to_T 0.0001 // 1 Gaus = 10000 Tesla
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

int init_accel(gnublin_i2c *port);

int init_gyro(gnublin_i2c *port);

int init_magnet(gnublin_i2c *port);

int read_accel(gnublin_i2c *port, float *x_axis, float *y_axis, float *z_axis);

int read_gyro(gnublin_i2c *port, float *temp, float *x_axis, float *y_axis, float *z_axis);

int read_magnet(gnublin_i2c *port, float *x_axis, float *y_axis, float *z_axis);

#endif
 //SENSOR_STICK_DRIVER_MAIN_H
