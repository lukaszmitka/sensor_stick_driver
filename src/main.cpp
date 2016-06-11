//
// Created by lukasz on 07.06.16.
//
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


#include <iostream>
#include <ros/ros.h>
#include "gnublin.h"

using namespace std;

unsigned char i2c_buffer;
unsigned char accel_buffer[6];
unsigned char gyro_buffer[8];
unsigned char magnet_buffer[6];
float linear_acceleration[3];
float rotational_acceleration[3];
float temperature;
float magnet_orientation[3];
unsigned int tmp_axis;
unsigned int lsb, msb;

int init_accel(gnublin_i2c *port);

int init_gyro(gnublin_i2c *port);

int init_magnet(gnublin_i2c *port);

int read_accel(gnublin_i2c *port, float *x_axis, float *y_axis, float *z_axis);

int read_gyro(gnublin_i2c *port, float *temp, float *x_axis, float *y_axis, float *z_axis);

int read_magnet(gnublin_i2c *port, float *x_axis, float *y_axis, float *z_axis);

int main(int argc, char **argv) {
   //ros::init(argc, argv, "ahrs_node");
   //ros::NodeHandle node;
   //ros::spin();
   //MadgwickAHRSupdate(1, 1, 1, 1, 1, 1, 1, 1, 1);

   gnublin_i2c i2c;
   init_accel(&i2c);
   init_gyro(&i2c);
   init_magnet(&i2c);
   sleep(1);

   for (unsigned char i = 0; i < 10; i++) {
      read_accel(&i2c, &linear_acceleration[0], &linear_acceleration[1], &linear_acceleration[2]);
      read_gyro(&i2c, &temperature, &rotational_acceleration[0], &rotational_acceleration[1],
                &rotational_acceleration[2]);
      read_magnet(&i2c, &magnet_orientation[0], &magnet_orientation[1], &magnet_orientation[2]);
      //cout << "Temp: " << temperature << endl;
      //printf("i = %d, byte: "BYTE_TO_BINARY_PATTERN"\n", (signed char)i, BYTE_TO_BINARY(
      //      (unsigned
      // char) i));
      usleep(5000);
   }


   /*while (node.ok()) {
         rate.sleep();
      }*/
   return 0;
}


int init_accel(gnublin_i2c *port) {
   // set the address of the slave
   if (port->setAddress(ADXL345_ADDRESS) == -1) {
      return -1;
   }
   i2c_buffer = 0b00000000; // last two bits: 11 -> +-16g; 00-> +-2g
   if (port->send(ADXL_DATA_FORMAT, &i2c_buffer, 1) == -1) {
      return -2;
   }

   i2c_buffer = 0b00001011; // 200Hz
   if (port->send(ADXL_REGISTER_BW_RATE, &i2c_buffer, 1) == -1) {
      return -3;
   }

   // Power ON
   i2c_buffer = ADXL_PWRCTL_MEASURE;
   if (port->send(ADXL_REGISTER_PWRCTL, &i2c_buffer, 1) == -1) {
      return -4;
   }
   return 0;
}

int read_accel(gnublin_i2c *port, float *x_axis, float *y_axis, float *z_axis) {
   // set the address of the slave
   if (port->setAddress(ADXL345_ADDRESS) == -1) {
      return -1;
   }
   if (port->receive(0x32, accel_buffer, 6) == -1) {
      return -2;
   }
   // x_axis
   lsb = accel_buffer[0];
   msb = (signed char) accel_buffer[1];
   tmp_axis = lsb + (msb << 8);
   *x_axis = ((signed int) tmp_axis) * ADXL_SCALE_FACTOR;

   // y_axis
   lsb = accel_buffer[2];
   msb = (signed char) accel_buffer[3];
   tmp_axis = lsb + (msb << 8);
   *y_axis = ((signed int) tmp_axis) * ADXL_SCALE_FACTOR;

   // z_axis
   lsb = accel_buffer[4];
   msb = (signed char) accel_buffer[5];
   tmp_axis = lsb + (msb << 8);
   *z_axis = ((signed int) tmp_axis) * ADXL_SCALE_FACTOR;

   //cout << "x: " << *x_axis << ", y: " << *y_axis << ", z: " << *z_axis << endl;
   return 0;
}

int init_gyro(gnublin_i2c *port) {
   // set the address of the slave you want to read/write
   if (port->setAddress(ITG_3200_ADDRESS) == -1) {
      return -1;
   }
   i2c_buffer = ITG_3200_DLPF_FS_DATA;
   if (port->send(ITG_3200_DLPF_FS_REG, &i2c_buffer, 1) == -1) {
      return -2;
   }

   i2c_buffer = ITG_3200_SR_DIV_DATA;
   if (port->send(ITG_3200_SR_DIV_REG, &i2c_buffer, 1) == -1) {
      return -3;
   }

   i2c_buffer = ITG_3200_POWER_DATA;
   if (port->send(ITG_3200_POWER_REG, &i2c_buffer, 1) == -1) {
      return -4;
   }

   if (port->receive(ITG_3200_WHO_AM_I, &i2c_buffer, 1) == -1) {
      return -5;
   }
   cout << "Gyro ID: " << i2c_buffer << endl;
   printf("Gyro ID: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(i2c_buffer));
   return 0;
}

int read_gyro(gnublin_i2c *port, float *temp, float *x_axis, float *y_axis, float *z_axis) {
   if (port->setAddress(ITG_3200_ADDRESS) == -1) {
      return -1;
   }
   if (port->receive(ITG_3200_SENS_REG, gyro_buffer, 8) == -1) {
      return -2;
   }

   // temp
   lsb = gyro_buffer[1];
   msb = gyro_buffer[0];
   tmp_axis = lsb + (msb << 8);
   //printf("Temp: "BYTE_TO_BINARY_PATTERN" ", BYTE_TO_BINARY(gyro_buffer[0]));
   //printf(""BYTE_TO_BINARY_PATTERN" ", BYTE_TO_BINARY(gyro_buffer[1]));
   *temp = 35.0f + ((tmp_axis + 13200) & 0xFFFF) / 280.0f;

   // x_axis
   lsb = gyro_buffer[3];
   msb = (signed char) gyro_buffer[2];
   tmp_axis = lsb + (msb << 8);
   *x_axis = ((signed int) tmp_axis) / ITG_3200_SCALE_FACTOR;

   // y_axis
   lsb = gyro_buffer[5];
   msb = (signed char) gyro_buffer[4];
   tmp_axis = lsb + (msb << 8);
   *y_axis = ((signed int) tmp_axis) / ITG_3200_SCALE_FACTOR;

   // z_axis
   lsb = gyro_buffer[7];
   msb = (signed char) gyro_buffer[6];
   tmp_axis = lsb + (msb << 8);
   *z_axis = ((signed int) tmp_axis) / ITG_3200_SCALE_FACTOR;
   return 0;
}

int init_magnet(gnublin_i2c *port) {
   // set the address of the slave you want to read/write
   if (port->setAddress(HMC5883L_ADDRESS) == -1) {
      return -1;
   }
   i2c_buffer = HMC5883L_CONF_DATA_A;
   if (port->send(HMC5883L_CONF_REG_A, &i2c_buffer, 1) == -1) {
      return -2;
   }

   i2c_buffer = HMC5883L_CONF_DATA_B;
   if (port->send(HMC5883L_CONF_REG_B, &i2c_buffer, 1) == -1) {
      return -3;
   }

   i2c_buffer = HMC5883L_MODE_DATA;
   if (port->send(HMC5883L_MODE_REG, &i2c_buffer, 1) == -1) {
      return -4;
   } else if (port->receive(HMC5883L_MODE_REG, &i2c_buffer, 1) == -1) {
      return -4;
   } else {
      printf("Mode: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(i2c_buffer));
   }

   if (port->receive(HMC5883L_STATUS_REG, &i2c_buffer, 1) == -1) {
      return -5;
   }
   cout << "Magnetometer status: " << i2c_buffer;
   printf("; "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(i2c_buffer));

   return 0;
}

int read_magnet(gnublin_i2c *port, float *x_axis, float *y_axis, float *z_axis) {
   if (port->setAddress(HMC5883L_ADDRESS) == -1) {
      return -1;
   }
   if (port->receive(HMC5883L_SENS_REG, magnet_buffer, 6) == -1) {
      return -2;
   }

   // x_axis
   lsb = magnet_buffer[1];
   msb = (signed char) magnet_buffer[0];
   tmp_axis = lsb + (msb << 8);
   printf("Gyro x: "BYTE_TO_BINARY_PATTERN" ", BYTE_TO_BINARY(magnet_buffer[0]));
   printf(""BYTE_TO_BINARY_PATTERN" ", BYTE_TO_BINARY(magnet_buffer[1]));
   *x_axis = ((signed int) tmp_axis) * HMC5883L_SCALE;
   printf("; %f, ", *x_axis);

   // y_axis
   lsb = magnet_buffer[5];
   msb = (signed char) magnet_buffer[4];
   tmp_axis = lsb + (msb << 8);
   printf(", y: "BYTE_TO_BINARY_PATTERN" ", BYTE_TO_BINARY(magnet_buffer[4]));
   printf(""BYTE_TO_BINARY_PATTERN" ", BYTE_TO_BINARY(magnet_buffer[5]));
   *y_axis = ((signed int) tmp_axis) * HMC5883L_SCALE;
   printf("; %f, ", *y_axis);

   // z_axis
   lsb = magnet_buffer[3];
   msb = (signed char) magnet_buffer[2];
   tmp_axis = lsb + (msb << 8);
   printf(", z: "BYTE_TO_BINARY_PATTERN" ", BYTE_TO_BINARY(magnet_buffer[2]));
   printf(""BYTE_TO_BINARY_PATTERN" ", BYTE_TO_BINARY(magnet_buffer[3]));
   *z_axis = ((signed int) tmp_axis) * HMC5883L_SCALE;
   printf("; %f\n", *z_axis);

   return 0;
}
