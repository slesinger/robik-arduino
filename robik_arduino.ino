/*
 * The MIT License (MIT)
 * Copyright (c) 2015 Honza Slesinger
 */

#include <Servo.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "CalLib.h"
#include <EEPROM.h>
#include <ros.h>
#include "robik/GenericStatus.h"
#include "robik/GenericControl.h"
#include "robik/ArmControl.h"
#include "robik/VelocityControl.h"
#include "std_msgs/UInt16.h"
#include "robik.h"
#include "robik_util.h"
#include "robik_arm.h"
#include "robik_move.h"
#include "robik_imu.h"
//#include "robik_park.h"
#include "robot_config.h"
#include "robik_api.h"


ros::NodeHandle nh;

void genericMessageListener(const robik::GenericControl& msg);
void velocityMessageListener(const robik::VelocityControl& msg);
void lidarrpmMessageListener(const std_msgs::UInt16& msg);
ros::Subscriber<robik::GenericControl> sub_generic_control("robik_generic_control", &genericMessageListener);
ros::Subscriber<robik::VelocityControl> sub_velocity_control("robik_velocity_control", &velocityMessageListener);
ros::Subscriber<std_msgs::UInt16> sub_lidarrpm_control("rpms", &lidarrpmMessageListener);
void setArmPower(bool status);
bool getArmPower();

robik::GenericStatus status_msg;
ros::Publisher pub_status("robik_status", &status_msg);

unsigned long odomMillisSinceLastUpdate = 0;
bool powerJetsonWasPoweredOn = false;
bool motionDetector = false;
bool motionDetectorPublished = true;
int lidar_curr_pwm = 0;
int lidar_curr_pwmd = 99;
int lidar_curr_pwmr = 98;
unsigned long last_lidar_update = 0;

void status_msg_clean () {
	charBuf[0] = '\0';
	status_msg.log_count = 0;
	status_msg.status_code_length = 0;
	status_msg.status_param_length = 0;
}

void jetsonPower() {

	if (digitalRead(PIN_JETSONPOWER) == LOW) {
		digitalWrite(PIN_PETRPOWER, LOW); // Jetson was powered down (was up before). Poweroff whole robot
	}
}


/*--------------------SETUP()------------------------*/
void setup() {

        //Utils
        setup_util(&status_msg);

	//Power
	pinMode(PIN_PETRPOWER, OUTPUT);   // this pin needs to be enabled to keep power on
	digitalWrite(PIN_PETRPOWER, HIGH); // turn on power
	pinMode(PIN_JETSONPOWER, INPUT);  // read power-on status of Jetson
	pinMode(PIN_CHARGER_BUTTON, OUTPUT);  // read power-on status of Jetson

	//Arm
	setup_arm(&nh);

	//Light
	pinMode(PIN_LED, OUTPUT);
	digitalWrite(PIN_LED, LOW);
	
	//Lidar
	pinMode(PIN_LIDAR_PWM, OUTPUT);
	std_msgs::UInt16 nula;// = new std_msgs::UInt16();
	nula.data = 0;
	lidar_curr_pwm = LIDAR_INIT_PWM;
	lidarrpmMessageListener(nula);

	//IMU
	setup_imu();

	//ROS node
	long baud = 500000;
	nh.getHardware()->setBaud(baud); //57600 115200 500000, need change in robik.launch as well
	nh.initNode();
	nh.subscribe(sub_generic_control);
	nh.subscribe(sub_arm_control);
	nh.subscribe(sub_velocity_control);
	nh.subscribe(sub_lidarrpm_control);
	nh.advertise(pub_status);

	while(!nh.connected()) {
		jetsonPower();
		nh.spinOnce();
	}
	nh.loginfo("Arduino connected");

	//Move base
	setup_move();

	//motion detector
	pinMode(PIN_MOTION_DETECTOR, INPUT);

}



/*---------------------- LOOP() ---------------------*/
void loop() {


	status_msg.header.stamp = nh.now();

	unsigned long now = millis();
	status_msg.odom_millisSinceLastUpdate = now - odomMillisSinceLastUpdate;
	odomMillisSinceLastUpdate = now;

	//Move base
	loop_move(status_msg);

	//arm
	loop_arm(status_msg);

	//lidar
      	//if ( (last_lidar_update + LIDAR_POWEROFF_TIMEOUT) < millis() ) {
	//  analogWrite(PIN_LIDAR_PWM, 0);
	//}

	//motion detector
	status_msg.motion_detector = motionDetector;
	motionDetectorPublished = true;
	
	//motion detector
	if (digitalRead(PIN_MOTION_DETECTOR) == HIGH) {
		motionDetectorPublished = false;
		motionDetector = true;
	}
	else {
		if (motionDetectorPublished == true) motionDetector = false;
	}

	//Parking
	//loop_park(status_msg);

	//IMU
	loop_imu(status_msg);

	//thigs to be done frequently
	jetsonPower();

	//publish
	pub_status.publish(&status_msg);
	nh.spinOnce();
	status_msg_clean();
	delay(2);
} //end of loop




/*------------------------ Listeners -------------------*/

void genop_head_pose(const robik::GenericControl& msg) {

	//head
		//int val = map(msg.gen_param2, -1000, 1000, 50, 165);
		//servoHeadPitch.write(val);

}

void setLED(const robik::GenericControl& msg) {
	digitalWrite(PIN_LED, (msg.gen_param1 == 1) ? HIGH : LOW);
}

void setDPin(const robik::GenericControl& msg) {
	digitalWrite(msg.gen_param1, (msg.gen_param2 == 1) ? HIGH : LOW);
}

void genericMessageListener(const robik::GenericControl& msg) {

	if (msg.gen_operation == OPERATION_HEAD_POSE) {
		genop_head_pose(msg);
	}
	if (msg.gen_operation == OPERATION_SET_PARKING_PHASE) {
	  //setParkingPhase(msg);
	}
	if (msg.gen_operation == OPERATION_SET_LED) {
		setLED(msg);
	}
	if (msg.gen_operation == OPERATION_SET_DPIN) {
		setDPin(msg);
	}
	if (msg.gen_operation == OPERATION_SET_ARMPOWER) {
		setArmPower((msg.gen_param1 == 1) ? true : false);
	}

}

void lidarrpmMessageListener(const std_msgs::UInt16& msg) {
  int pwm_diff = 0;
  uint16_t i_rpm = msg.data;

  last_lidar_update = millis();
  if ( (i_rpm >= 60) && (i_rpm <=350) ) {
    int rpm_diff = LIDAR_TARGET_RPM - i_rpm;
    rpm_diff = constrain(rpm_diff, -50, 50);
    pwm_diff = map(rpm_diff,  -50, 50,  -5, 5);
    lidar_curr_pwm += pwm_diff;

  }
  else { // else we do not have rpm value or we do not trust the value
    lidar_curr_pwm = LIDAR_INIT_PWM; 
  }
	
  analogWrite(PIN_LIDAR_PWM, lidar_curr_pwm);
}




