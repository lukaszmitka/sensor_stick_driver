//
// Created by lukasz on 07.06.16.
//
#define BOARD RASPBERRY_PI
#define ADXL345_ADDRESS 0x53

#include <iostream>
#include <ros/ros.h>
#include "gnublin.h"

using namespace std;

int main(int argc, char **argv) {
   //ros::init(argc, argv, "ahrs_node");
   //ros::NodeHandle node;
   //ros::spin();
   //MadgwickAHRSupdate(1, 1, 1, 1, 1, 1, 1, 1, 1);

   gnublin_i2c i2c;


   i2c.setAddress(ADXL345_ADDRESS); // set the address of the slave you want to read/write


   unsigned char RxBuf = 0;
   unsigned char buf = 0x08;

   i2c.send(0x2d, &buf, 1);
   //i2c.send(0x12, buffer, 2);   //send 2 bytes from buffer to register-address 0x12

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