#ifndef ROBOT_CONFIG
#define ROBOT_CONFIG
//===============

/* Bumper */
#define PIN_BUMPER 39

/* Menu controls */
#define PIN_MENU_MENU 49
#define PIN_MENU_OK 51
#define PIN_MENU_CANCEL 50
#define PIN_MENU_UP 53
#define PIN_MENU_DOWN 52

/* Motors */
#define PIN_MOTOR_LEFT_ENA 2
#define PIN_MOTOR_LEFT_0 26
#define PIN_MOTOR_LEFT_1 27

#define PIN_MOTOR_RIGHT_ENA 3
#define PIN_MOTOR_RIGHT_0 28
#define PIN_MOTOR_RIGHT_1 29

#define MOTOR_MIN 2
#define MOTOR_MAX 199 //verified that this is highest speed
//#define MOTOR_MIN_ROTATION 8/60 //[rot/sec]
//#define MOTOR_MAX_ROTATION 80/60 //[rot/sec]

#define WHEEL_DIA_MM 100
#define TICK_LENGTH_MM 1.9 //2.456
#define AXIAL_LENGTH_MM 305.0

#define MAX_VEL 0.4 // m/s
#define MAX_TH 2.4  //RAD/s - if one wheel is stopeed and other works full speed
#define MIN_TH 0

/* Sonar */
#define PIN_USOUND_ECHO 42
#define PIN_USOUND_TRIG 43
#define SONAR_NUM     1  // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

/* Parking Light Sensor */
#define PIN_PARK_SENS_INNER A6
#define PIN_PARK_SENS_OUTER A7

/* Head */
#define PIN_MOTION_DETECTOR 38
#define PIN_LED 40

/* Oko */
#define PIN_OKO_SERVO_YAW 8
#define PIN_OKO_SERVO_PITCH 9

/* Odometry encoder */
#define PIN_ODOM_LEFT 18
#define PIN_ODOM_RIGHT 19
#define INT_ODOM_LEFT 5
#define INT_ODOM_RIGHT 4

/* Power */
#define PIN_RELAY 30
#define PIN_PETRPOWER 22
#define PIN_JETSONPOWER 23
#define PIN_CHARGER_BUTTON 24  //is set at action_move.py
//==============

/* Arm */
#define PIN_SERVO_SENSE_YAW A1
#define PIN_SERVO_SENSE_SHOULDER A2
#define PIN_SERVO_SENSE_ELBOW A3
#define PIN_SERVO_SENSE_ROLL A4
#define PIN_SERVO_SENSE_CLAMP A5
#define SERVO_SENSE_INTERVAL 1

#define PIN_ARM_FORE_CLAMP 36
#define PIN_ARM_BACK_CLAMP 37

#define ARM_TIME_MAX_ENABLED 60000 //milliseconds, how long can arm power be enabled at maximum, it powers off automatically after this
#define ARM_TIME_MIN_BREAK 60000 //milliseconds, minimum amount of time that arm needs to rest

//!!!! nastavit jestli se to nehodi na desce. Jinak zjisteno, ze tyto piny jsou volne.
#define PIN_SERVO_SENSE_SHOULDER2 A8 
#define PIN_MOTOR_SHOULDER_0 44
#define PIN_MOTOR_SHOULDER_1 45


#endif
