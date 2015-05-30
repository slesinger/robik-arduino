/*
 * robik_state.h
 *
 *  Created on: Jun 28, 2013
 *      Author: honza
 * This header file contains robot's state that is supposed to be shared between modules of this driver
 */

#ifndef ROBIK_STATE_H_
#define ROBIK_STATE_H_

bool powerJetsonWasPoweredOn = false;

bool bumperFront = false; //when setting to true set also published to false. Do not set to false if published is false
bool bumperFrontPublished = true;

bool motionDetector = false;
bool motionDetectorPublished = true;

int odom_ticks_left = 0;
int odom_ticks_right = 0;
unsigned long odomMillisSinceLastUpdate = 0;

float ultrasoundBack = 0;


#endif /* ROBIK_STATE_H_ */
