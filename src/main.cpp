//
// Created by lukasz on 07.06.16.
//
#define BOARD RASPBERRY_PI
#define ADXL345_ADDRESS 0x53
#define ADXL_REGISTER_PWRCTL 0x2D
#define ADXL_REGISTER_BW_RATE 0x2C
#define ADXL_DATA_FORMAT 0x31
#define ADXL_PWRCTL_MEASURE 0x8

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
int acceleration[3];
unsigned int tmp_axis;
unsigned int lsb, msb;

int init_accel(gnublin_i2c *port);

int read_accel(gnublin_i2c *port, int *x_axis, int *y_axis, int *z_axis);

int main(int argc, char **argv) {
   //ros::init(argc, argv, "ahrs_node");
   //ros::NodeHandle node;
   //ros::spin();
   //MadgwickAHRSupdate(1, 1, 1, 1, 1, 1, 1, 1, 1);

   gnublin_i2c i2c;
   init_accel(&i2c);


   //unsigned char RxBuf = 0;

   for (unsigned char i = 0; i < 100; i++) {
      read_accel(&i2c, &acceleration[0], &acceleration[1], &acceleration[2]);
      usleep(5000);
      //printf("i = %d, byte: "BYTE_TO_BINARY_PATTERN"\n", (signed char)i, BYTE_TO_BINARY(
      //      (unsigned
      // char) i));

      /*while (node.ok()) {
         rate.sleep();
      }*/}
   return 0;
}


int init_accel(gnublin_i2c *port) {
   port->setAddress(ADXL345_ADDRESS); // set the address of the slave you want to read/write

   i2c_buffer = 0b00000000; // last two bits: 11 -> +-16g; 00-> +-2g
   port->send(ADXL_DATA_FORMAT, &i2c_buffer, 1);

   i2c_buffer = 0b00001011; // 200Hz
   port->send(ADXL_REGISTER_BW_RATE, &i2c_buffer, 1);

   // Power ON
   i2c_buffer = ADXL_PWRCTL_MEASURE;
   port->send(ADXL_REGISTER_PWRCTL, &i2c_buffer, 1);
}

int read_accel(gnublin_i2c *port, int *x_axis, int *y_axis, int *z_axis) {

   port->receive(0x32, accel_buffer, 6);
   // x_axis
   lsb = accel_buffer[0];
   msb = (signed char) accel_buffer[1];
   tmp_axis = lsb + (msb << 8);
   *x_axis = (signed int) tmp_axis;

   // y_axis
   lsb = accel_buffer[2];
   msb = (signed char) accel_buffer[3];
   tmp_axis = lsb + (msb << 8);
   *y_axis = (signed int) tmp_axis;

   // z_axis
   lsb = accel_buffer[4];
   msb = (signed char) accel_buffer[5];
   tmp_axis = lsb + (msb << 8);
   *z_axis = (signed int) tmp_axis;

   cout << "x: " << *x_axis << ", y: " << *y_axis << ", z: " << *z_axis << endl;
}