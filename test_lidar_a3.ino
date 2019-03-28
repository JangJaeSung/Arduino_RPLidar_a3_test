#include <stdio.h>
#include <Servo.h>
#include <rplidar.h>
#include <ros.h>

#include <sensor_msgs/LaserScan.h>

#define KP 1.5

ros::NodeHandle nh;

// You need to create an driver instance


// Change the pin mapping based on your needs.
#define RPLIDAR_MOTOR  3 // The PWM pin for control the speed of RPLIDAR's motor.

Servo steerServo;
Servo throttleMotor;
RPLidar lidar(Serial, RPLIDAR_MOTOR);
RPLidarPacket packet;

const int rpm = 600;

sensor_msgs::LaserScan scan_msg;
ros::Publisher rp_msg("rp_msg", &scan_msg);

void setup() {
  // bind the RPLIDAR driver to the arduino hardware serial
  nh.initNode();
  Serial.begin(57600);
  //lidar.begin(Serial);
  nh.spinOnce();
  // set pin modes
  pinMode(13, OUTPUT);
  //pinMode(RPLIDAR_MOTOR, OUTPUT);
  lidar.setup(rpm);
  lidar.start();
}


void loop() {  
  nh.spinOnce();
  //lidar.startScan();
 // analogWrite(RPLIDAR_MOTOR, 255);
  
  float angle = 0;
  //angle = lidar.getCurrentPoint().angle;
  scan_msg.angle_min = angle;

  if( lidar.processAvailable(&packet) )
  {
    if(packet.data[0] > 0){
      digitalWrite(13, HIGH);
      delay(1000);
      digitalWrite(13, LOW);
      delay(1000);
    }
  }

  rp_msg.publish(&scan_msg);

  delay(1000);
  
}
