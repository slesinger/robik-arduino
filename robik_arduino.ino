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
#include "robik_arm.h"
#include "robik_move.h"
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

unsigned long range_timer50;

//IMU
RTIMU *imu;                                           // the IMU object
RTIMUSettings settings;                               // the settings object
void read_IMU();


//DISPLAY_INTERVAL sets the rate at which results are displayed
#define DISPLAY_INTERVAL  300                         // interval between pose displays

unsigned long lastDisplay;
unsigned long lastRate;

//determine the sign of a value
template <typename type>
type sign(type value) {
	return type((value>0)-(value<0));
}

template <typename type>
type signNZ(type value) {
	return (value < 0) ? -1 : 1;
}


//move.h
void intr_left_encoder();
void intr_right_encoder();
int motor_vel_to_pwm(double vel);

/*------------------Utils----------------------------*/
#define LOG_STR_LEN 150
char charBuf[LOG_STR_LEN + 1];
char strtmp[LOG_STR_LEN + 1];

void init_variables() {

	estimated_clamp_pos.orig_time.sec = 0;
	estimated_clamp_pos.orig_time.nsec = 0;
	estimated_clamp_pos.time_to_complete = 0;
	estimated_clamp_pos.orig_pos = 0;
	estimated_clamp_pos.target_pos = 0;
	estimated_clamp_pos.corrected = false;

	status_msg.status_code = (int32_t *) malloc(20 * sizeof(int32_t));
	
}

void log_string(String str) {

	unsigned int str_length = str.length();

	if (str_length > 0) {

		if (str_length > LOG_STR_LEN)
			str_length = LOG_STR_LEN;

		str.toCharArray(charBuf, str_length + 1);
		status_msg.log_message = &charBuf[0];
		status_msg.log_count++;

	}

}

void log_chars(char *str) {

	if (strlen(str) > 0) {
		strcpy(charBuf, str);
		status_msg.log_message = &charBuf[0];
		status_msg.log_count++;

	}

}

void status_msg_clean () {
	charBuf[0] = '\0';
	status_msg.log_count = 0;
	status_msg.status_code_length = 0;
	status_msg.status_param_length = 0;
}

void add_status_code (int32_t code) {
	if (status_msg.status_code_length < 20) {
		status_msg.status_code[status_msg.status_code_length] = code;
		status_msg.status_code_length++;
	}
}

void jetsonPower() {

	if (digitalRead(PIN_JETSONPOWER) == LOW) {
		digitalWrite(PIN_PETRPOWER, LOW); // Jetson was powered down (was up before). Poweroff whole robot
	}
}


/*------------------State----------------------------*/
#include "robik_state.h"

/*--------------------SETUP()------------------------*/
void setup() {

	//Power
	pinMode(PIN_PETRPOWER, OUTPUT);   // this pin needs to be enabled to keep power on
	digitalWrite(PIN_PETRPOWER, HIGH); // turn on power
	pinMode(PIN_JETSONPOWER, INPUT);  // read power-on status of Jetson
	pinMode(PIN_CHARGER_BUTTON, OUTPUT);  // read power-on status of Jetson

	init_variables();

	//Arm
	setup_arm(&nh);

	//LED
	pinMode(PIN_LED, OUTPUT);
	digitalWrite(PIN_LED, LOW);

	//IMU init
	int errcode;
	Wire.begin();
	imu = RTIMU::createIMU(&settings);  // create the imu object

	if ((errcode = imu->IMUInit()) < 0) {
		//#Serial.print("Failed to init IMU: "); Serial.println(errcode);
	}

	imu->getCalibrationValid();

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

	range_timer50 = 11;

	//odometry encoders
	pinMode(PIN_ODOM_LEFT, INPUT);
	digitalWrite(PIN_ODOM_LEFT, HIGH);       // turn on pullup resistor
	pinMode(PIN_ODOM_RIGHT, INPUT);
	digitalWrite(PIN_ODOM_RIGHT, HIGH);       // turn on pullup resistor
	attachInterrupt(INT_ODOM_LEFT, intr_left_encoder, CHANGE);
	attachInterrupt(INT_ODOM_RIGHT, intr_right_encoder, CHANGE);

	//motion detector
	pinMode(PIN_MOTION_DETECTOR, INPUT);

	//velocity
	pinMode(PIN_MOTOR_LEFT_ENA, OUTPUT); //Motor left
	pinMode(PIN_MOTOR_LEFT_0, OUTPUT);
	pinMode(PIN_MOTOR_LEFT_1, OUTPUT);
	pinMode(PIN_MOTOR_RIGHT_ENA, OUTPUT); //Motor right
	pinMode(PIN_MOTOR_RIGHT_0, OUTPUT);
	pinMode(PIN_MOTOR_RIGHT_1, OUTPUT);

	//menu control
	pinMode(PIN_MENU_MENU, INPUT);
	digitalWrite(PIN_MENU_MENU, HIGH); // turn on pullup resistors
	pinMode(PIN_MENU_OK, INPUT);
	digitalWrite(PIN_MENU_OK, HIGH); // turn on pullup resistors
	pinMode(PIN_MENU_CANCEL, INPUT);
	digitalWrite(PIN_MENU_CANCEL, HIGH); // turn on pullup resistors
	pinMode(PIN_MENU_UP, INPUT);
	digitalWrite(PIN_MENU_UP, HIGH); // turn on pullup resistors
	pinMode(PIN_MENU_DOWN, INPUT);
	digitalWrite(PIN_MENU_DOWN, HIGH); // turn on pullup resistors

}


unsigned int menu_lag = 0;
uint8_t menu_controls(){
  uint8_t res = 0;

  if (menu_lag > 3) {
		menu_lag = 0;
		if (digitalRead(PIN_MENU_MENU) == LOW)
			res |= MASK_MENU_MENU;
		if (digitalRead(PIN_MENU_OK) == LOW)
			res |= MASK_MENU_OK;
		if (digitalRead(PIN_MENU_CANCEL) == LOW)
			res |= MASK_MENU_CANCEL;
		if (digitalRead(PIN_MENU_UP) == LOW)
			res |= MASK_MENU_UP;
		if (digitalRead(PIN_MENU_DOWN) == LOW)
			res |= MASK_MENU_DOWN;
  }
  else {
		menu_lag++;
  }
  return res;
}


/*---------------------- LOOP() ---------------------*/
void loop() {


	status_msg.header.stamp = nh.now();

	//menu controls
	status_msg.menu_controls = menu_controls();

	//odom
	status_msg.odom_ticksLeft = motor_left_dir * odom_ticks_left;
	status_msg.odom_ticksRight = motor_right_dir * odom_ticks_right;
	odom_ticks_left = 0;
	odom_ticks_right = 0;
	unsigned long now = millis();
	status_msg.odom_millisSinceLastUpdate = now - odomMillisSinceLastUpdate;
	odomMillisSinceLastUpdate = now;

	//wheels are expected to move but robot is stopped
	velocity_control_LR();

	//arm
	loop_arm(status_msg);

	//motion detector
	status_msg.motion_detector = motionDetector;
	motionDetectorPublished = true;
	
	loop_park(status_msg);

	//IMU
	status_msg.imu_gyro_bias_valid = imu->IMUGyroBiasValid();  //do not move IMU if false

	refreshMotorJoints();

	//publish
	pub_status.publish(&status_msg);
	status_msg_clean();


	//thigs to be done frequently
	read_IMU();
	jetsonPower();
	
	nh.spinOnce();

	//motion detector
	if (digitalRead(PIN_MOTION_DETECTOR) == HIGH) {
		motionDetectorPublished = false;
		motionDetector = true;
	}
	else {
		if (motionDetectorPublished == true) motionDetector = false;
	}

} //end of loop



void read_IMU() {

	if (imu->IMURead()) { // get the latest data if ready yet
		RTVector3 gyro_v3 = (RTVector3&)imu->getGyro();
		status_msg.imu_angular_velocity_v3_x = gyro_v3.x();
		status_msg.imu_angular_velocity_v3_y = gyro_v3.y();
		status_msg.imu_angular_velocity_v3_z = gyro_v3.z();

		RTVector3 accel_v3 = (RTVector3&)imu->getAccel();
		status_msg.imu_linear_acceleration_v3_x = accel_v3.x();
		status_msg.imu_linear_acceleration_v3_y = accel_v3.y();
		status_msg.imu_linear_acceleration_v3_z = accel_v3.z();

		RTVector3 compass_v3 = (RTVector3&)imu->getCompass();
		status_msg.imu_compass_v3_x = compass_v3.x();
		status_msg.imu_compass_v3_y = compass_v3.y();
		status_msg.imu_compass_v3_z = compass_v3.z();
	}

}


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

/*
Postup ladeni
- zapoj potenciometr serva do analogu a zapis min a max hodnotu
- odladit smer otaceni, pak odstranit 100ms stop  //konfiguruj PWM stejne jako ohm
- vyzkouset jestli shoulder2 se hybe stejne jako shoulder1
*/

//TODO pridej parametr effort
void setMotorJoint(int pin0, int pin1, int pinsense, int target_ohm) {

	int current_ohm = analogRead(pinsense);
	//add_status_code(current_ohm);

	int a0 = LOW;  //stop implicitly
	int a1 = LOW;
	if (current_ohm > target_ohm) {
		a0 = HIGH;
	}
	else if (current_ohm < target_ohm) {
		a1 = HIGH;
	}
	/*digitalWrite(pin0, a0);*/
	/*digitalWrite(pin1, a1);*/

	//delay(100);   //zastav motor pro 100ms dokud neodladis smer
	/*digitalWrite(pin0, LOW);*/
	/*digitalWrite(pin1, LOW);*/
}

void refreshMotorJoints() {
//TODO rozeber servo a predelej to ne lm298n
  //setMotorJoint(PIN_MOTOR_SHOULDER_0, PIN_MOTOR_SHOULDER_1, PIN_SERVO_SENSE_SHOULDER2, shoulder2);
  //add more joints here
}


//Input received from diff_drive_controller
void velocityMessageListener(const robik::VelocityControl& msg) {
	req_motor_left = constrain(msg.motor_left, -MAX_VEL, MAX_VEL);
	req_motor_right = constrain(msg.motor_right, -MAX_VEL, MAX_VEL);
	velocity_control_LR();
}

//odometry encoders
void intr_left_encoder() {

	odom_ticks_left++;
	odom_ticks_left_since_cmd++;
}

void intr_right_encoder() {

	odom_ticks_right++;
	odom_ticks_right_since_cmd++;
}
