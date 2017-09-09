/*
 * robik_api.h
 *
 *  Created on: Jun 28, 2013
 *      Author: honza
 * This header file is shared between robik_driver and arduino sketch
 */

#ifndef ROBIK_API_H_
#define ROBIK_API_H_

#define OPERATION_HEAD_POSE 1
#define OPERATION_SET_PARKING_PHASE 2
#define OPERATION_SET_LED 3
#define OPERATION_SET_DPIN 4
#define OPERATION_SET_ARMPOWER 5

#define ARM_CANON_MIN 1
#define ARM_CANON_MAX 1000

//Hodnoty PWM
#define ARM_MIN_CLAMP 1200
#define ARM_MAX_CLAMP 650
#define ARM_MIN_ROLL 2250
#define ARM_MAX_ROLL 730
#define ARM_MIN_ELBOW 2250
#define ARM_MAX_ELBOW 500
#define ARM_MIN_SHOULDER 950
#define ARM_MAX_SHOULDER 1830
#define ARM_MIN_YAW 2130
#define ARM_MAX_YAW 600

//Hodnoty Resistoru (OHM)
#define ARM_RES_MIN_CLAMP 223
#define ARM_RES_MAX_CLAMP 122
#define ARM_RES_MIN_ROLL 413
#define ARM_RES_MAX_ROLL 103
#define ARM_RES_MIN_ELBOW 545
#define ARM_RES_MAX_ELBOW 44
#define ARM_RES_MIN_SHOULDER 170
#define ARM_RES_MAX_SHOULDER 423
#define ARM_RES_MIN_SHOULDER2 1
#define ARM_RES_MAX_SHOULDER2 1024
#define ARM_RES_MIN_YAW 424
#define ARM_RES_MAX_YAW 111

//Init ohm values (parked arm)  //TODO update values after calibration, this was just estimation
#define ARM_RES_INIT_CLAMP 202
#define ARM_RES_INIT_ROLL 291
#define ARM_RES_INIT_ELBOW 545
#define ARM_RES_INIT_SHOULDER 264
#define ARM_RES_INIT_YAW 393

//degrees
#define ARM_DEG_MIN_CLAMP 0		//0.0
#define ARM_DEG_MAX_CLAMP 40		//0.052 RAD
#define ARM_DEG_MIN_ROLL -90		//-1.571
#define ARM_DEG_MAX_ROLL 90		//1.571
#define ARM_DEG_MIN_ELBOW -110		//-2.094
#define ARM_DEG_MAX_ELBOW 40		//0.611
#define ARM_DEG_MIN_SHOULDER 0		//-0.20
#define ARM_DEG_MAX_SHOULDER 120	//1.7
#define ARM_DEG_MIN_YAW -107		//-2.05
#define ARM_DEG_MAX_YAW 63		//1.092

//Menu controls
#define MASK_MENU_MENU 1
#define MASK_MENU_OK 2
#define MASK_MENU_CANCEL 4
#define MASK_MENU_UP 8
#define MASK_MENU_DOWN 16


#endif /* ROBIK_API_H_ */
