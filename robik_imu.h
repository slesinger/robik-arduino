/*
 * robik_imu.h
 *
 *  Created on: Dec 12, 2015
 *      Author: Honza Slesinger
 * This header file is shared between robik_driver and arduino sketch
 */

#ifndef ROBIK_IMU_H_
#define ROBIK_IMU_H_


//IMU
RTIMU *imu;                                           // the IMU object
RTIMUSettings settings;                               // the settings object
void read_IMU();

void setup_imu() {
	//IMU init
	int errcode;
	Wire.begin();
	imu = RTIMU::createIMU(&settings);  // create the imu object

	if ((errcode = imu->IMUInit()) < 0) {
		//#Serial.print("Failed to init IMU: "); Serial.println(errcode);
	}

	imu->getCalibrationValid();
}

void loop_imu(robik::GenericStatus& status_msg) {
	status_msg.imu_gyro_bias_valid = imu->IMUGyroBiasValid();  //do not move IMU if false

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

#endif /* ROBIK_IMU_H_ */
