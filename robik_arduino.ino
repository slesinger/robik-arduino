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
int motor_left_dir = 1; //direction for motor expected to go. 1 means forward
int motor_right_dir = 1;
double req_motor_left = 0; //store required wheel velocity
double req_motor_right = 0;
double old_motor_left = 0; //store to compare if speed changed since last cmd_vel
double old_motor_right = 0;
long odom_ticks_left_since_cmd = 0;
long odom_ticks_right_since_cmd = 0;
unsigned long millis_since_cmd = 0;

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
void velocity_control_VelTh(float vel, float th);
void velocity_control_LR();
int motor_vel_to_pwm(double vel);

//parking.h
#define PARK_BACKSPEED -0.17 //-0.1 necouva
#define PARK_TURNSPEED 0.9 //calibrated, do not put less, it won't work unless excitation energy will be provided
#define PARK_BACKTURNSPEED_SLOW_RATIO 0.2
#define PARKPHASE_NOPARKING 0
#define PARKPHASE_NAVIGATING 1
#define PARKPHASE_ROTATING 2
#define PARKPHASE_BACKWARD 3
#define PARKPHASE_PARKED 4
	//0: normal operation;
	//1: navigating to base;
	//2: rotating to find laser;
	//3: going backwards;
	//4: final stage going backwards
int state_parking = 0;
int park_last_spotted = 0; //0: no value; 1: inner; -1: outer

void park_rotating(bool inner, bool outer);
void park_backward(bool inner, bool outer);
void park_parked(bool inner, bool outer);
void handle_parking();

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


	//parking photo sensor
	status_msg.parkSens_inner = analogRead(PIN_PARK_SENS_INNER);
	status_msg.parkSens_outer = analogRead(PIN_PARK_SENS_OUTER);


	//IMU
	status_msg.imu_gyro_bias_valid = imu->IMUGyroBiasValid();  //do not move IMU if false

	refreshMotorJoints();

	//publish
	pub_status.publish(&status_msg);
	status_msg_clean();


	//thigs to be done frequently
	read_IMU();
	handle_parking();
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

	loop_frequent_arm();

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

void park_rotating(bool inner, bool outer) {

	if (inner || outer) {
		//stop rotation
		velocity_control_VelTh(PARK_BACKSPEED, 0);
		state_parking = PARKPHASE_BACKWARD;
		park_last_spotted = 1; //if beam is lost, turn reverse first to find it
		// ROS_INFO("Laser beam found. Switching to PARKPHASE_BACKWARD");
	}

}

void park_backward(bool inner, bool outer) {

	//adjust speed
	float parking_speed = PARK_BACKSPEED; //default ful speed
	if ( ultrasoundBack < 10 ) { // getting as close as 10cm, slow down
		// ROS_DEBUG("Jsem blizko 20cm. Brzdim na pulku");
		parking_speed = PARK_BACKSPEED / 2;
	}
	if ( ultrasoundBack < 2 ) {  // <2cm stop
		parking_speed = 0;
		state_parking = PARKPHASE_PARKED;
	}

	float dir = 0;  // >0 toci se doleva
	if (inner && outer)
		dir = 0;
	else if (inner == true && outer == false) {
		dir = +1 * PARK_BACKTURNSPEED_SLOW_RATIO;
		park_last_spotted = 1;
	}
	else if (inner == false && outer == true) {
		dir = -1 * PARK_BACKTURNSPEED_SLOW_RATIO;
		park_last_spotted = -1;
	}
	else if (inner == false && outer == false && park_last_spotted == 1) {
		dir = +1 * 1.3 * PARK_BACKTURNSPEED_SLOW_RATIO;
	}
	else if (inner == false && outer == false && park_last_spotted == -1) {
		dir = -1 * 1.3 * PARK_BACKTURNSPEED_SLOW_RATIO;
	}
	else if (inner == false && outer == false && park_last_spotted == 0) {
		dir = 1;
		parking_speed = 0;
		state_parking = 0;
		// ROS_ERROR("Parking laser beam lost. Switching to manual parking.");
	}

	velocity_control_VelTh(parking_speed, dir * PARK_TURNSPEED);
	// ROS_INFO("B I:%d O:%d US: %f lst: %d | spd: %f tw %f",	inner, outer, laserscan_msg.ranges[0], park_last_spotted, parking_speed, dir * PARK_TURNSPEED);

}

void park_parked(bool inner, bool outer) {
	velocity_control_VelTh(0, 0);
	// ROS_INFO("Parked");
	state_parking = PARKPHASE_NOPARKING;
}

void handle_parking() {

	if (state_parking > PARKPHASE_NOPARKING) {

		int ps_inner = analogRead(PIN_PARK_SENS_INNER);
		int ps_outer = analogRead(PIN_PARK_SENS_OUTER);
		bool inner = false;
		bool outer = false;
		if (ps_inner < 320)
			inner = true;
		if (ps_outer < 280)
			outer = true;

		//ROS_INFO("PS status: { inner: %d, outer: %d }", inner, outer);

		switch (state_parking) {
			case PARKPHASE_ROTATING:
				park_rotating(inner, outer);
				break;
			case PARKPHASE_BACKWARD:
				park_backward(inner, outer);
				break;
			case PARKPHASE_PARKED:
				park_parked(inner, outer);
				break;
		}
	}

}

/*------------------------ Listeners -------------------*/

void genop_head_pose(const robik::GenericControl& msg) {

	//head
		//int val = map(msg.gen_param2, -1000, 1000, 50, 165);
		//servoHeadPitch.write(val);

}

void setParkingPhase(const robik::GenericControl& msg) {
	state_parking = msg.gen_param1;
	if (msg.gen_param1 == PARKPHASE_ROTATING) {
		velocity_control_VelTh(0, 1 * PARK_TURNSPEED);
	}
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

// vel: linear velocity [m/sec]
// theta: angular velocity [rad/sec]
void velocity_control_VelTh(float vel, float th) {

	//limit to standard max values on input
	vel = constrain(vel, -MAX_VEL, MAX_VEL);
	th = constrain(th, -MAX_TH, MAX_TH);

	//combine speed and rotation and set as required speed for boot wheels
	req_motor_left  = vel - ((AXIAL_LENGTH_MM/1000 / 2) * th);
	req_motor_right = vel + ((AXIAL_LENGTH_MM/1000 / 2) * th);

	//limit combined speed to maximum available
	double coef = 1; 
	if (abs(req_motor_left) > MAX_VEL)
		coef = req_motor_left / MAX_VEL;
	if (abs(req_motor_right) > MAX_VEL)
		coef = req_motor_right / MAX_VEL;

	req_motor_left = req_motor_left / abs(coef);
	req_motor_right = req_motor_right / abs(coef);

	velocity_control_LR();

}


int motor_vel_to_pwm(double vel) {
	return vel / MAX_VEL * MOTOR_MAX;
}

int is_equal_3decplaces(double a, double b) {
		long long ai = a * 1000;
		long long bi = b * 1000;
		return ai == bi;
}

//input is global variable req_motor_left and req_motor_right, [meter/sec], max speed is normalized
void velocity_control_LR() {

	//if velocity command has changed
	if ( (!is_equal_3decplaces(req_motor_left, old_motor_left)) || 
	     (!is_equal_3decplaces(req_motor_right, old_motor_right))
	    ||
	     (req_motor_left == 0 && req_motor_right == 0)
	) {
		//remove correction, for new velocity command use only estimated speeds
		old_motor_left = req_motor_left;
		old_motor_right = req_motor_right;
		millis_since_cmd = millis();
		odom_ticks_left_since_cmd = 0;
		odom_ticks_right_since_cmd = 0;
	}

	//number of ticks that the robot is behind. Can be also < 0 which means the robot is ahead of required
	long millis_now = millis();
	long correction_motor_left  = long( (double)(millis_now - millis_since_cmd)/1000.0 * req_motor_left  * 1000 / TICK_LENGTH_MM ) - odom_ticks_left_since_cmd;
	long correction_motor_right = long( (double)(millis_now - millis_since_cmd)/1000.0 * req_motor_right * 1000 / TICK_LENGTH_MM ) - odom_ticks_right_since_cmd;

	//include correction <0 - ow; MOTOR_MAX + ow> where ow is an overflow
	int corrected_motor_left  = motor_vel_to_pwm(abs(req_motor_left))  + constrain(correction_motor_left  * 5, -MOTOR_MAX/5, MOTOR_MAX/5);
	int corrected_motor_right = motor_vel_to_pwm(abs(req_motor_right)) + constrain(correction_motor_right * 5, -MOTOR_MAX/5, MOTOR_MAX/5);
	//add_status_code(corrected_motor_left);
	//add_status_code(corrected_motor_right);

	//remap if speed of any wheel overflows 0 or MOTOR_MAX
	if (corrected_motor_left < MOTOR_MIN)
		corrected_motor_left = 0;
	if (corrected_motor_right < MOTOR_MIN)
		corrected_motor_right = 0;

	int ow_left = 0;
	int ow_right = 0;
	int ow = 0;
	if (corrected_motor_left > MOTOR_MAX)
		ow_left = corrected_motor_left - MOTOR_MAX;
	if (corrected_motor_right > MOTOR_MAX)
		ow_right = corrected_motor_right - MOTOR_MAX;

	//subtract common overflow for both wheels
	ow = max(ow_left, ow_right);
	corrected_motor_left -= ow;
	corrected_motor_right -= ow;

	//derive direction from original speed
	//left
	int a0 = LOW;  //stop implicitly
	int a1 = LOW;
	if (req_motor_left < 0) {
		a0 = HIGH;
		motor_left_dir = -1;
	}
	if (req_motor_left > 0) {
		a1 = HIGH;
		motor_left_dir = 1;
	}
	digitalWrite(PIN_MOTOR_LEFT_0, a0);
	digitalWrite(PIN_MOTOR_LEFT_1, a1);
	analogWrite(PIN_MOTOR_LEFT_ENA, constrain(corrected_motor_left, 0, MOTOR_MAX));
	//add_status_code(correction_motor_left);
	//add_status_code(corrected_motor_left);

	//right
	a0 = LOW;
	a1 = LOW;
	if (req_motor_right < 0) {
		a0 = HIGH;
		motor_right_dir = -1;
	}
	if (req_motor_right > 0) {
		a1 = HIGH;
		motor_right_dir = 1;
	}
	digitalWrite(PIN_MOTOR_RIGHT_0, a0);
	digitalWrite(PIN_MOTOR_RIGHT_1, a1);
	analogWrite(PIN_MOTOR_RIGHT_ENA, constrain(corrected_motor_right, 0, MOTOR_MAX));
	//add_status_code(correction_motor_right);
	//add_status_code(corrected_motor_right);
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
