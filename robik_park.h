/*
 * robik_park.h
 *
 *  Created on: Dec 12, 2015
 *      Author: Honza Slesinger
 * This header file is shared between robik_driver and arduino sketch
 */

#ifndef ROBIK_PARK_H_
#define ROBIK_PARK_H_

#include "robik/GenericStatus.h"
#include "robik_move.h"

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
void setParkingPhase(const robik::GenericControl& msg);

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
	/*if ( ultrasoundBack < 10 ) { // getting as close as 10cm, slow down
		// ROS_DEBUG("Jsem blizko 20cm. Brzdim na pulku");
		parking_speed = PARK_BACKSPEED / 2;
		}
	if ( ultrasoundBack < 2 ) {  // <2cm stop
		parking_speed = 0;
		state_parking = PARKPHASE_PARKED;
		}*/

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



void loop_park(robik::GenericStatus& status_msg) {
	//parking photo sensor
	status_msg.parkSens_inner = analogRead(PIN_PARK_SENS_INNER);
	status_msg.parkSens_outer = analogRead(PIN_PARK_SENS_OUTER);

	//handle parking, frequently
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

void setParkingPhase(const robik::GenericControl& msg) {
	state_parking = msg.gen_param1;
	if (msg.gen_param1 == PARKPHASE_ROTATING) {
		velocity_control_VelTh(0, 1 * PARK_TURNSPEED);
	}
}

#endif /* ROBIK_PARK_H_ */
