#include <AccelStepper.h>
#include <ros.h>
#include <std_msgs/Float32.h>

// Define motor interface type
#define MotorInterfaceType 1

// Pin assignments for stepper motors
const int stepPin1 = 13;
const int dirPin1 = 12;
const int stepPin2 = 11;
const int dirPin2 = 10;
const int stepPin3 = 9;
const int dirPin3 = 8;

// Stepper motor instances
AccelStepper stepper1(MotorInterfaceType, stepPin1, dirPin1);
AccelStepper stepper2(MotorInterfaceType, stepPin2, dirPin2);
AccelStepper stepper3(MotorInterfaceType, stepPin3, dirPin3);

// ROS NodeHandle
ros::NodeHandle nh;

// ROS Messages
std_msgs::Float32 stepper1_pos_msg;
std_msgs::Float32 stepper2_pos_msg;
std_msgs::Float32 stepper3_pos_msg;

// ROS Publishers
ros::Publisher pub_stepper1_pos("stepper1/position", &stepper1_pos_msg);
ros::Publisher pub_stepper2_pos("stepper2/position", &stepper2_pos_msg);
ros::Publisher pub_stepper3_pos("stepper3/position", &stepper3_pos_msg);

// Target positions for stepper motors
float targetPos1 = 0;
float targetPos2 = 0;
float targetPos3 = 0;

// Callback functions for subscribers
void targetPos1Callback(const std_msgs::Float32& msg) {
  static float lastTargetPos1 = -99999;
  if (abs(msg.data - lastTargetPos1) > 1.0) { // Process only significant changes
    targetPos1 = msg.data;
    stepper1.moveTo(static_cast<long>(targetPos1));
    lastTargetPos1 = msg.data;
  }
}

// Callback function for stepper 2
void targetPos2Callback(const std_msgs::Float32& msg) {
  static float lastTargetPos2 = -99999;
  if (abs(msg.data - lastTargetPos2) > 1.0) { // Process only significant changes
    targetPos2 = msg.data;
    stepper2.moveTo(static_cast<long>(targetPos2));
    
    // Automatically update stepper 3 to be -1 * stepper 2
    targetPos3 = 1 * targetPos2;
    stepper3.moveTo(static_cast<long>(targetPos3));

    lastTargetPos2 = msg.data;
  }
}

// ROS Subscribers
ros::Subscriber<std_msgs::Float32> sub_targetPos1("targetPos1", targetPos1Callback);
ros::Subscriber<std_msgs::Float32> sub_targetPos2("targetPos2", targetPos2Callback);

void setup() {
  nh.initNode();

  // Advertise publishers
  nh.advertise(pub_stepper1_pos);
  nh.advertise(pub_stepper2_pos);
  nh.advertise(pub_stepper3_pos);

  // Subscribe to target position topics
  nh.subscribe(sub_targetPos1);
  nh.subscribe(sub_targetPos2);

  // Initialize stepper motors
  stepper1.setMaxSpeed(2000);
  stepper1.setAcceleration(500);

  stepper2.setMaxSpeed(1000);
  stepper2.setAcceleration(500);

  stepper3.setMaxSpeed(1000);
  stepper3.setAcceleration(500);

  // Set initial positions
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
}

void loop() {
  static unsigned long lastPublishTime = 0;

  // Run steppers to their target positions
  stepper1.run();
  stepper2.run();
  stepper3.run();

  // Publish current positions at a reduced frequency (e.g., every 100 ms)
  unsigned long currentTime = millis();
  if (currentTime - lastPublishTime > 100) {
    stepper1_pos_msg.data = stepper1.currentPosition();
    stepper2_pos_msg.data = stepper2.currentPosition();
    stepper3_pos_msg.data = stepper3.currentPosition();

    pub_stepper1_pos.publish(&stepper1_pos_msg);
    pub_stepper2_pos.publish(&stepper2_pos_msg);
    pub_stepper3_pos.publish(&stepper3_pos_msg);

    lastPublishTime = currentTime;
  }

  // Spin ROS node
  nh.spinOnce();
}
