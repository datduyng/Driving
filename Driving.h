/*
Author:	CheeTown Liew
Version of June 10, 2017
Version: 1.0
feature:
  This algorithm is designed to drive the robot and run with precise displacement
  using motor encoder, the algorithm includes simple proportion controller and
  displacement-encoder count convertor, speed comparator and path tracking.

  Micro-controller Board: Arduino Mega
  Motor Driver: Sabertooth Motor Driver


Modified by: Dat Nguyen
Date :06/23/18
Version: 2.0
added feature:
  - added accelerometer.
  - this program will allow mobile platform move accordingly to encoder tick and
  Accerlorometer at the same time.
  - This program will allow mobile platform turn according to IMU reading.

spec:
  - 227:1 Metal Gearmotor 25Dx56L mm MP 12v with 48 CPR Encoer(popolu.com)
  - Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055(adafruit)
    - https://www.adafruit.com/product/2472
  - HC-SR04 ultrasonic sensor
*/

#ifndef Driving_h
#define Driving_h

#include <Arduino.h>
#include <SabertoothSimplified.h>

#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


//define constant
#define ENCODERREVOLUTION 227.1 * 48
#define INCH2C 227.1 * 48 / ( 3 * 3.141592653589 * 2 * 2 )
#define COUNTER2INCHE ( 3 * 3.14159265359 * 2 * 2) / (227.1 * 48)
#define WHEELWIDTH 11.6875

#define IMU_DELAY (100)
#define M_PI 3.141592653589

//define default encoder pins
#define ECLA    2 //LEFT encoder channel A
#define ECRA    3 //RIGHT encoder channel A
#define ECLB    4 //LEFT encoder channel B
#define ECRB    5 //RIGHT encoder channel B
#define EPOWERL   52  //LEFT encoder Vcc
#define EPOWERR   53  //RIGHT encoder Vcc

//define default motor driver communication
#define MOTOR_DRIVER  14    //Using Tx_3 pin (Serial3)

#define R_MOTOR_MAX 110
#define L_MOTOR_MAX 120
//define controller constant
#define K1    0.440945		//LEFT controller constant
#define K2    0.5			//RIGHT controller constant
#define V	1.3			//speed controller constant
#define I	0.7		//integral controller constant

// define PID constant
#define kp 1.0
#define ki 1.0
#define kd 1.0

//declare global coordinates
extern float global_x;
extern float global_y;
extern float local_x;
extern float local_y;
extern int16_t global_orientation;

// global coordinate according to IMU
extern float global_IMU_x;
extern float global_IMU_y;
extern float local_IMU_x;
extern float local_IMU_y;
extern int16_t global_IMU_orientation;
extern int16_t local_IMU_orientation;

// accel and gyro reading
extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;

//declare encoder counter
extern volatile int64_t ECL;    //encoder counter - LEFT
extern volatile int64_t ECLM;  //encoder counter - LEFT MEMORY
extern volatile int64_t ECR;    //encoder counter - RIGHT
extern volatile int64_t ECRM;  //encoder counter - RIGHT MEMORY

//declare parameters
extern bool debug1;

//private functions
static void counter1(void);
static void counter2(void);

//public functions--------------------------------------------------------
void dinit (void);		//default initiator, needed before using the functions from this library
//void dinit ( uint8_t& pins );
void driveto( float distance );
void steer(int16_t toAngle );

void debugMode(void);
void debug(bool);


//converter functions
int64_t D2C( float distance );
float C2D (int64_t encoderCount);
int64_t R2C ( int16_t angle );
int16_t C2R ( int64_t encoderCount );


/*
 * THis function convert radian to degree)
 * return int16_t degree
 * param float radian
 */
int16_t R2D (float radian);

// IMU function
bool imuInit(void);

// debug IMU function
void displaySensorDetails(void);
void displaySensorStatus(void);
void displayCalStatus(void);

#endif
