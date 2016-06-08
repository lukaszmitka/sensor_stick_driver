//
// Created by lukasz on 07.06.16.
//
#define BOARD RASPBERRY_PI
#define ADXL345_ADDRESS 0x53
#define ADXL_REGISTER_PWRCTL 0x2D
#define ADXL_PWRCTL_MEASURE 0x8

#include <iostream>
#include <ros/ros.h>
#include "gnublin.h"

using namespace std;

unsigned char i2c_buffer;

int init_accel(gnublin_i2c *port);

int main(int argc, char **argv) {
   //ros::init(argc, argv, "ahrs_node");
   //ros::NodeHandle node;
   //ros::spin();
   //MadgwickAHRSupdate(1, 1, 1, 1, 1, 1, 1, 1, 1);

   gnublin_i2c i2c;

   init_accel(&i2c);

   unsigned char RxBuf = 0;

   for (int i = 0; i < 10; i++) {
      i2c.receive(0x32, &RxBuf, 1);
      cout << "Received: " << (uint16_t) RxBuf << endl;
      //i2c.receive(0x23, RxBuf, 3);  // read from  tegister-address 0x23 3 bytes and store them in
      // RxBuf
      sleep(1);
   }


   /*while (node.ok()) {
      rate.sleep();
   }*/
   return 0;
};

int init_accel(gnublin_i2c *port) {
   port->setAddress(ADXL345_ADDRESS); // set the address of the slave you want to read/write
   // example use:
   //i2c.send(0x12, &buffer, 2);   //send 2 bytes from buffer to register-address 0x12
   i2c_buffer = ADXL_PWRCTL_MEASURE;
   port->send(ADXL_REGISTER_PWRCTL, &i2c_buffer, 1);
}

