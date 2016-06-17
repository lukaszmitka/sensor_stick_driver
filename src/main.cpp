//
// Created by lukasz on 07.06.16.
//

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "gnublin.h"
#include "main.h"

using namespace std;

unsigned char i2c_buffer;
unsigned char accel_buffer[6];
unsigned char gyro_buffer[8];
unsigned char magnet_buffer[6];
float linear_acceleration[3];
float rotation_speed[3];
float temperature;
float magnet_orientation[3];
unsigned int tmp_axis;
unsigned int lsb, msb;
signed int s_msb, s_tmp;

int main(int argc, char **argv) {
   ros::init(argc, argv, "sensor_stick_driver_node");
   ros::NodeHandle node("~");

// node.getParam("param_name", verbose)

   ros::Publisher imu_pub = node.advertise<sensor_msgs::Imu>("/imu/data_raw", 5);
   ros::Publisher mag_pub = node.advertise<sensor_msgs::MagneticField>("/imu/mag", 5);

   ros::Rate loop_rate(200);

   gnublin_i2c i2c;
   init_accel(&i2c);
   init_gyro(&i2c);
   init_magnet(&i2c);
   sleep(1);

   while (node.ok()) {
      sensor_msgs::Imu imu;
      read_accel(&i2c, &linear_acceleration[0], &linear_acceleration[1], &linear_acceleration[2]);
      read_gyro(&i2c, &temperature, &rotation_speed[0], &rotation_speed[1],
                &rotation_speed[2]);
      read_magnet(&i2c, &magnet_orientation[0], &magnet_orientation[1], &magnet_orientation[2]);

      imu.angular_velocity.x = rotation_speed[0];
      imu.angular_velocity.y = rotation_speed[1];
      imu.angular_velocity.z = rotation_speed[2];
      imu.linear_acceleration.x = linear_acceleration[0];
      imu.linear_acceleration.y = linear_acceleration[1];
      imu.linear_acceleration.z = linear_acceleration[2];
      imu.header.frame_id = "imu";
      imu_pub.publish(imu);

      sensor_msgs::MagneticField magField;
      magField.magnetic_field.x = magnet_orientation[0];
      magField.magnetic_field.y = magnet_orientation[1];
      magField.magnetic_field.z = magnet_orientation[2];
      mag_pub.publish(magField);
      loop_rate.sleep();
   }
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
   *x_axis = g_to_m_s * ((signed int) tmp_axis) * ADXL_SCALE_FACTOR;

   // y_axis
   lsb = accel_buffer[2];
   msb = (signed char) accel_buffer[3];
   tmp_axis = lsb + (msb << 8);
   *y_axis = g_to_m_s * ((signed int) tmp_axis) * ADXL_SCALE_FACTOR;

   // z_axis
   lsb = accel_buffer[4];
   msb = (signed char) accel_buffer[5];
   tmp_axis = lsb + (msb << 8);
   *z_axis = g_to_m_s * ((signed int) tmp_axis) * ADXL_SCALE_FACTOR;

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
   s_msb = (signed char) gyro_buffer[0];
   s_tmp = lsb + (s_msb << 8);
   *temp = 35.0f + ((s_tmp + 13200) / 280.0f);

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
      // printf("Mode: "BYTE_TO_BINARY_PATTERN"\n", BYTE_TO_BINARY(i2c_buffer));
   }

   if (port->receive(HMC5883L_STATUS_REG, &i2c_buffer, 1) == -1) {
      return -5;
   }

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
   *x_axis = Gs_to_T * ((signed int) tmp_axis) * HMC5883L_SCALE;

   // y_axis
   lsb = magnet_buffer[5];
   msb = (signed char) magnet_buffer[4];
   tmp_axis = lsb + (msb << 8);
   *y_axis = Gs_to_T * ((signed int) tmp_axis) * HMC5883L_SCALE;

   // z_axis
   lsb = magnet_buffer[3];
   msb = (signed char) magnet_buffer[2];
   tmp_axis = lsb + (msb << 8);
   *z_axis = Gs_to_T * ((signed int) tmp_axis) * HMC5883L_SCALE;

   return 0;
}
