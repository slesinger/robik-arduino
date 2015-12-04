# robik_arduino
Arduino drivers for Robik robot
ROS package elsewhere

```
/*
 * On the Arduino Mega we have 6 timers and 15 PWM outputs:
 * Pins 4 and 13: controlled by timer0 (used by clock)
 * Pins 11 and 12: controlled by timer1 (Servo library)
 * Pins 9 and10: controlled by timer2 (NewPing)
 * Pin 2, 3 and 5: controlled by timer 3 (free)
 * Pin 6, 7 and 8: controlled by timer 4 (free)
 * Pin 46, 45 and 44:: controlled by timer 5 (free)
 *
 * PWM pins 2-13, 44-46. Free 9: 2,3,5,6,7,8,44,45,46
 *
 * IMU MPU9150 uses library from https://github.com/richards-tech/RTIMULib-Arduino.git
 */
 
/*
LIDAR
Pinout for data
Red      +5V
Brown    LDS_RX   (not needed)
Orange   LDS_TX   > pin 15 (1000ohm? resistor)
Black    GND

Pinout for motor
Red      PWR      > 5V (40ohm resistor)
Black    GND

*/
```