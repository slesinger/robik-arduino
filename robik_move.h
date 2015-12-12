/*
 * robik_move.h
 *
 *  Created on: Dec 12, 2015
 *      Author: Honza Slesinger
 * This header file is shared between robik_driver and arduino sketch
 */

#ifndef ROBIK_MOVE_H_
#define ROBIK_MOVE_H_

void velocity_control_VelTh(float vel, float th);
void velocity_control_LR();

int motor_left_dir = 1; //direction for motor expected to go. 1 means forward
int motor_right_dir = 1;
double req_motor_left = 0; //store required wheel velocity
double req_motor_right = 0;
double old_motor_left = 0; //store to compare if speed changed since last cmd_vel
double old_motor_right = 0;
long odom_ticks_left_since_cmd = 0;
long odom_ticks_right_since_cmd = 0;
int odom_ticks_left = 0;
int odom_ticks_right = 0;
unsigned long millis_since_cmd = 0;

void intr_left_encoder();
void intr_right_encoder();
int motor_vel_to_pwm(double vel);


void setup_move() {
	//odometry encoders
	pinMode(PIN_ODOM_LEFT, INPUT);
	digitalWrite(PIN_ODOM_LEFT, HIGH);       // turn on pullup resistor
	pinMode(PIN_ODOM_RIGHT, INPUT);
	digitalWrite(PIN_ODOM_RIGHT, HIGH);       // turn on pullup resistor
	attachInterrupt(INT_ODOM_LEFT, intr_left_encoder, CHANGE);
	attachInterrupt(INT_ODOM_RIGHT, intr_right_encoder, CHANGE);

	//velocity
	pinMode(PIN_MOTOR_LEFT_ENA, OUTPUT); //Motor left
	pinMode(PIN_MOTOR_LEFT_0, OUTPUT);
	pinMode(PIN_MOTOR_LEFT_1, OUTPUT);
	pinMode(PIN_MOTOR_RIGHT_ENA, OUTPUT); //Motor right
	pinMode(PIN_MOTOR_RIGHT_0, OUTPUT);
	pinMode(PIN_MOTOR_RIGHT_1, OUTPUT);


}

void loop_move(robik::GenericStatus& status_msg) {
	//odom
	status_msg.odom_ticksLeft = motor_left_dir * odom_ticks_left;
	status_msg.odom_ticksRight = motor_right_dir * odom_ticks_right;
	odom_ticks_left = 0;
	odom_ticks_right = 0;

	//wheels are expected to move but robot is stopped
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

#endif /* ROBIK_MOVE_H_ */
