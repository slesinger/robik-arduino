/*
 * robik_arm.h
 *
 *  Created on: Dec 12, 2015
 *      Author: Honza Slesinger
 * This header file is shared between robik_driver and arduino sketch
 */

#include <RunningMedian.h>

#ifndef ROBIK_ARM_H_
#define ROBIK_ARM_H_

void armMessageListener(const robik::ArmControl& msg);
ros::Subscriber<robik::ArmControl> sub_arm_control("robik_arm_control",
		&armMessageListener);

void armPreInit();
void armSetJointState(uint32_t clamp, uint32_t roll, uint32_t elbow,
		uint32_t shoulder, uint32_t yaw, uint32_t time_to_complete);
unsigned long servoSenseTimer;
FastRunningMedian<unsigned int, 32, 0> servoSenseMedian_yaw;
FastRunningMedian<unsigned int, 32, 0> servoSenseMedian_shoulder;
FastRunningMedian<unsigned int, 32, 0> servoSenseMedian_elbow;
FastRunningMedian<unsigned int, 32, 0> servoSenseMedian_roll;
FastRunningMedian<unsigned int, 32, 0> servoSenseMedian_clamp;
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

void setup_arm() {
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

void loop_arm() {
	status_msg.arm_enabled = getArmPower();
	status_msg.arm_yaw = servoSenseMedian_yaw.getMedian();
	status_msg.arm_shoulder = servoSenseMedian_shoulder.getMedian();
	status_msg.arm_elbow = servoSenseMedian_elbow.getMedian();
	status_msg.arm_roll = servoSenseMedian_roll.getMedian();
	status_msg.arm_clamp = servoSenseMedian_clamp.getMedian();

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
	//servo sense
	if (millis() >= servoSenseTimer) {      // Is it this sensor's time to ping?
		servoSenseTimer += SERVO_SENSE_INTERVAL;
		servoSenseMedian_yaw.addValue(analogRead(PIN_SERVO_SENSE_YAW));
		servoSenseMedian_shoulder.addValue(
				analogRead(PIN_SERVO_SENSE_SHOULDER));
		servoSenseMedian_elbow.addValue(analogRead(PIN_SERVO_SENSE_ELBOW));
		servoSenseMedian_roll.addValue(analogRead(PIN_SERVO_SENSE_ROLL));
		servoSenseMedian_clamp.addValue(analogRead(PIN_SERVO_SENSE_CLAMP));
	}
}

void checkClampAndStop() {
//time_to_complete [ms]
	ros::Time now = nh.now();
	uint32_t milliseconds_since_start = ros_time_diff(
			estimated_clamp_pos.orig_time, now);

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

bool getArmPower() {
	return (arm_enabled_time != 0) ? true : false;
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
		estimated_clamp_pos.orig_time = nh.now();
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



#endif /* ROBIK_ARM_H_ */
