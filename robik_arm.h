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
ros::Subscriber<robik::ArmControl> sub_arm_control("robik_arm_control",
		&armMessageListener);

void armPreInit();
void armSetJointState(uint32_t clamp, uint32_t roll, uint32_t elbow,
		uint32_t shoulder, uint32_t yaw, uint32_t time_to_complete);
unsigned long servoSenseTimer;
uint32_t shoulder2 = 512;
estimated_pos_t estimated_clamp_pos;
void refreshMotorJoints();
unsigned long arm_enabled_time = 0;
unsigned long arm_disabled_time = 1;
uint32_t old_clamp = 0;
uint32_t old_roll = 0;
uint32_t old_elbow = 0;
uint32_t old_shoulder = 0;
uint32_t old_yaw = 0;

bool getArmPower() {
	return (arm_enabled_time != 0) ? true : false;
}

//Return time difference in milliseconds
uint32_t ros_time_diff(ros::Time before, ros::Time after) {
	uint32_t sec = after.sec - before.sec;
	uint32_t nsec = after.nsec - before.nsec;

	if (nsec < 0) {
		nsec = 1000 + nsec;
	}

	return sec * 1000 + nsec / 1000000;
}

//-1: t2<t1
// 0: t2==t1
// 1: t2>t1
int ros_time_cmp(ros::Time t1, ros::Time t2) {
	if (t1.sec == t2.sec && t1.nsec == t2.nsec)
		return 0;

	if (ros_time_diff(t1, t2) > 0)
		return 1;
	else
		return -1;
}

void checkClampAndStop() {
//time_to_complete [ms]
	ros::Time now = _nh->now();
	uint32_t milliseconds_since_start = ros_time_diff(estimated_clamp_pos.orig_time, now);

	if ((estimated_clamp_pos.orig_pos < estimated_clamp_pos.target_pos)
			&& (milliseconds_since_start
					< (estimated_clamp_pos.time_to_complete + 700))) { //clamp is being moved, try hard for extra 700
		double progress = milliseconds_since_start
				/ estimated_clamp_pos.time_to_complete;
		progress *= 0.9; //bring it back a bit to compensate late detection
		if (progress > 1)
			progress = 1;

		uint32_t estimatedClampPos = estimated_clamp_pos.orig_pos
				+ (uint32_t)(
						progress
								* (estimated_clamp_pos.target_pos
										- estimated_clamp_pos.orig_pos));
		//sprintf(strtmp, "SSS %lu | TTC %lu | ORG %lu | TRG %lu | PRG %lu | ECP %lu", milliseconds_since_start, estimated_clamp_pos.time_to_complete, estimated_clamp_pos.orig_pos, estimated_clamp_pos.target_pos, (uint32_t)(progress*100), estimatedClampPos); log_chars(strtmp);
		armSetJointState(estimatedClampPos, 0, 0, 0, 0, 80); //80ms
		estimated_clamp_pos.corrected = true; //prevent iteration for stopping clamp again
	}
	//else
	//sprintf(strtmp, "SSS %lu | TTC %lu | ORG %lu | TRG %lu ", milliseconds_since_start, estimated_clamp_pos.time_to_complete, estimated_clamp_pos.orig_pos, estimated_clamp_pos.target_pos); log_chars(strtmp);
}

void setArmPower(bool state) {
	if (state) {
		if (arm_disabled_time + ARM_TIME_MIN_BREAK < millis()) {
			digitalWrite(PIN_RELAY, HIGH);
			arm_enabled_time = millis();
			arm_disabled_time = 0;
		}
	}
	else {
		digitalWrite(PIN_RELAY, LOW);
		arm_enabled_time = 0;
		arm_disabled_time = millis();
	}
}

//replace by constrain arduino funtion after logging will no longer be needed
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
	Serial3.print("#10P1320T500\r\n"); delay(500);
	Serial3.print("#3P2250T500\r\n"); delay(500);
	Serial3.print("#4P1740T500\r\n"); delay(500);
	Serial3.print("#5P1091T500\r\n"); delay(500);
	Serial3.print("#1P1908T500\r\n"); delay(500);
	Serial3.print("#10P1280T500\r\n"); delay(500);

	Serial3.print("#11P700T1\r\n");  //set initial speed for shoulder 2
	shoulder2 = 512; //set initial position for shoulder2
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
		estimated_clamp_pos.orig_time = _nh->now();
		estimated_clamp_pos.time_to_complete = time_to_complete;
		estimated_clamp_pos.orig_pos = estimated_clamp_pos.target_pos;
		estimated_clamp_pos.target_pos = check_limits(clamp, ARM_MIN_CLAMP, ARM_MAX_CLAMP);
		estimated_clamp_pos.corrected = false;
	}

	if (roll != 0) {
		arm_roll = "#4P" + String(check_limits(roll, ARM_MIN_ROLL, ARM_MAX_ROLL), DEC);
	}

	if (elbow != 0) {
		arm_elbow = "#3P" + String(check_limits(elbow, ARM_MIN_ELBOW, ARM_MAX_ELBOW), DEC);
	}

	if (shoulder != 0) {
		arm_shoulder = "#10P" + String(check_limits(shoulder, ARM_MIN_SHOULDER, ARM_MAX_SHOULDER), DEC);
		shoulder2 = 512; //TODO map from shoulder
// otestuje servo   setMotorJoint(PIN_MOTOR_SHOULDER_0, PIN_MOTOR_SHOULDER_1, PIN_SERVO_SENSE_SHOULDER2, constrain(shoulder2, ARM_RES_MIN_SHOULDER2, ARM_RES_MAX_SHOULDER2));
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
	digitalWrite(PIN_RELAY, HIGH);
	armPreInit();
	digitalWrite(PIN_RELAY, LOW);  //turn off arm (as standby)

}

void loop_arm(robik::GenericStatus& status_msg) {
	status_msg.arm_enabled = getArmPower();
	status_msg.arm_yaw = analogRead(PIN_SERVO_SENSE_YAW);
	status_msg.arm_shoulder = analogRead(PIN_SERVO_SENSE_SHOULDER);
	status_msg.arm_elbow = analogRead(PIN_SERVO_SENSE_ELBOW);
	status_msg.arm_roll = analogRead(PIN_SERVO_SENSE_ROLL);
	status_msg.arm_clamp = analogRead(PIN_SERVO_SENSE_CLAMP);

	status_msg.arm_fore_clamp =
			digitalRead(PIN_ARM_FORE_CLAMP) == LOW ? true : false;
	status_msg.arm_back_clamp =
			digitalRead(PIN_ARM_BACK_CLAMP) == LOW ? true : false;
	if (estimated_clamp_pos.corrected == false
			&& (status_msg.arm_fore_clamp == true
					|| status_msg.arm_back_clamp == true))
		checkClampAndStop();

	//check arm power timeout to prevent overheat
	if ((getArmPower() == true)
			&& ((arm_enabled_time + ARM_TIME_MAX_ENABLED) < millis())) {
		setArmPower(false);
	}

}

void loop_frequent_arm() {
}




#endif /* ROBIK_ARM_H_ */
