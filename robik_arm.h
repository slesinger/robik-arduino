/*
 * robik_arm.h
 *
 *  Created on: Dec 12, 2015
 *      Author: Honza Slesinger
 * This header file is shared between robik_driver and arduino sketch
 */

#ifndef ROBIK_ARM_H_
#define ROBIK_ARM_H_

#include "robik/GenericStatus.h"
#include <robot_config.h>
#include <robik_api.h>

ros::NodeHandle *_nh;

void armMessageListener(const robik::ArmControl& msg);
ros::Subscriber<robik::ArmControl> sub_arm_control("robik_arm_control", &armMessageListener);

void armPreInit();
void armSetJointState(uint32_t clamp, uint32_t roll, uint32_t elbow, uint32_t shoulder, uint32_t yaw, uint32_t time_to_complete);
unsigned long servoSenseTimer;
void refreshMotorJoints();
unsigned long arm_enabled_time = 0;  //od kdy je zapnuty
unsigned long arm_disabled_time = 1; //od kdy je vypnuty
uint32_t old_clamp = 0;
uint32_t old_roll = 0;
uint32_t old_elbow = 0;
uint32_t old_shoulder = 0;
uint32_t old_yaw = 0;

//Return time difference in milliseconds
uint32_t ros_time_diff(ros::Time before, ros::Time after) {
	uint32_t sec = after.sec - before.sec;
	uint32_t nsec = after.nsec - before.nsec;

	if (nsec < 0) {
		nsec = 1000 + nsec;
	}

	return sec * 1000 + nsec / 1000000;
}

bool getArmPower() {
	return (arm_enabled_time != 0) ? true : false;
}

void setArmPower(bool state) {
	if (state == true) {
//		if (arm_disabled_time + ARM_TIME_MIN_BREAK < millis()) {
			digitalWrite(PIN_RELAY, HIGH); //zapni
			arm_enabled_time = millis();
			arm_disabled_time = 0;
//		}
	}
	else { //disable
		digitalWrite(PIN_RELAY, LOW); //vypni
		arm_enabled_time = 0;
		arm_disabled_time = millis();
	}
}

//replace by constrain arduino function after logging will no longer be needed
uint32_t check_limits(uint32_t val, uint32_t min, uint32_t max) {

    if (min > max) {
	uint32_t t = min;
	min = max;
	max = t;
    }

    if (val < min) {
	val = min;
    }
    if (val > max) {
	val = max;
    }

    return val;
}

void armPreInit() {
	Serial3.print("#10P1495T500\r\n"); delay(500);
	Serial3.print("#3P874T500\r\n"); delay(500);
	Serial3.print("#4P1485T500\r\n"); delay(500);
	Serial3.print("#5P1200T500\r\n"); delay(500);
	Serial3.print("#1P1974T500\r\n"); delay(500);
	//Serial3.print("#10P1730T500\r\n"); delay(500);
}


void armMessageListener(const robik::ArmControl& msg) {

	armSetJointState(msg.arm_clamp, msg.arm_roll, msg.arm_elbow, msg.arm_shoulder, msg.arm_yaw, msg.time_to_complete);

}

void armSetJointState(uint32_t clamp, uint32_t roll, uint32_t elbow, uint32_t shoulder, uint32_t yaw, uint32_t time_to_complete) {
	String arm_clamp = "";
	String arm_roll = "";
	String arm_elbow = "";
	String arm_shoulder = "";
	String arm_yaw = "";
add_status_code(9);
	//if arm power is off but a move request is received, enable power
	if ( (getArmPower() == false) &&
		(old_clamp != clamp ||
		old_roll != roll ||
		old_elbow != elbow ||
		old_shoulder != shoulder ||
		old_yaw != yaw)
	    )
	{
		setArmPower(true);
	}
	old_clamp = clamp;
	old_roll = roll;
	old_elbow = elbow;
	old_shoulder = shoulder;
	old_yaw = yaw;

	String time_to_complete_str = String(time_to_complete, DEC);
	if (clamp != 0) {
		arm_clamp = "#5P" + String(check_limits(clamp, ARM_MIN_CLAMP, ARM_MAX_CLAMP), DEC);
	}

	if (roll != 0) {
		arm_roll = "#4P" + String(check_limits(roll, ARM_MIN_ROLL, ARM_MAX_ROLL), DEC);
	}

	if (elbow != 0) {
		arm_elbow = "#3P" + String(check_limits(elbow, ARM_MIN_ELBOW, ARM_MAX_ELBOW), DEC);
	}

	if (shoulder != 0) {
		arm_shoulder = "#10P" + String(check_limits(shoulder, ARM_MIN_SHOULDER, ARM_MAX_SHOULDER), DEC);
	}

	if (yaw != 0) {
		arm_yaw = "#1P" + String(check_limits(yaw, ARM_MIN_YAW, ARM_MAX_YAW), DEC);
	}

	String cmd = String(arm_clamp + arm_roll + arm_elbow + arm_shoulder + arm_yaw + "T" + time_to_complete_str + "\r\n");
	//log_string(cmd);
	Serial3.print(cmd);

}


void setup_arm(ros::NodeHandle *nh) {

        _nh = nh;
	Serial3.begin(9600);
	delay(100);
	servoSenseTimer = millis();
	pinMode(PIN_ARM_FORE_CLAMP, INPUT);
	pinMode(PIN_ARM_BACK_CLAMP, INPUT);
	digitalWrite(PIN_ARM_FORE_CLAMP, HIGH); // turn on pullup resistors
	digitalWrite(PIN_ARM_BACK_CLAMP, HIGH); // turn on pullup resistors

	//power arm
	pinMode(PIN_RELAY, OUTPUT);
	setArmPower(true);
	armPreInit();
	//setArmPower(false); //as standby

}

void loop_arm(robik::GenericStatus& status_msg) {
	status_msg.arm_enabled = getArmPower();
	status_msg.arm_yaw = analogRead(PIN_SERVO_SENSE_YAW);
	status_msg.arm_shoulder = analogRead(PIN_SERVO_SENSE_SHOULDER);
	status_msg.arm_elbow = analogRead(PIN_SERVO_SENSE_ELBOW);
	status_msg.arm_roll = analogRead(PIN_SERVO_SENSE_ROLL);
	status_msg.arm_clamp = analogRead(PIN_SERVO_SENSE_CLAMP);

	status_msg.arm_fore_clamp = digitalRead(PIN_ARM_FORE_CLAMP) == LOW ? true : false;
	status_msg.arm_back_clamp = digitalRead(PIN_ARM_BACK_CLAMP) == LOW ? true : false;

	//check arm power timeout to prevent overheat
	if ((getArmPower() == true) && ((arm_enabled_time + ARM_TIME_MAX_ENABLED) < millis())) {
		//setArmPower(false);
	}

}


#endif /* ROBIK_ARM_H_ */
