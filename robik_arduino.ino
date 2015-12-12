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
#include "robik.h"
#include "robik_util.h"
#include "robik_arm.h"
#include "robik_move.h"
#include "robik_imu.h"
#include "robik_park.h"
#include "robot_config.h"
#include "robik_api.h"


ros::NodeHandle nh;

void genericMessageListener(const robik::GenericControl& msg);
void velocityMessageListener(const robik::VelocityControl& msg);
ros::Subscriber<robik::GenericControl> sub_generic_control("robik_generic_control", &genericMessageListener);
ros::Subscriber<robik::VelocityControl> sub_velocity_control("robik_velocity_control", &velocityMessageListener);
void setArmPower(bool status);
bool getArmPower();

robik::GenericStatus status_msg;
ros::Publisher pub_status("robik_status", &status_msg);

unsigned long odomMillisSinceLastUpdate = 0;
bool powerJetsonWasPoweredOn = false;
bool motionDetector = false;
bool motionDetectorPublished = true;


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

	//IMU
	setup_imu();

	//ROS node
	nh.getHardware()->setBaud(500000); //57600 115200 500000
	nh.initNode();
	nh.subscribe(sub_generic_control);
	nh.subscribe(sub_arm_control);
	nh.subscribe(sub_velocity_control);
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
	loop_park(status_msg);

	//IMU
	loop_imu(status_msg);

	//thigs to be done frequently
	jetsonPower();
	
	//publish
	pub_status.publish(&status_msg);
	status_msg_clean();
	nh.spinOnce();

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
		setParkingPhase(msg);
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


