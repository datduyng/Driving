
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
Date :07/25/18
Version: 2.0
added feature:
  - this program will allow mobile platform move accordingly to encoder tick and
  Accerlorometer at the same time.
  - This program will allow mobile platform turn according to IMU reading.

spec:
  - 227:1 Metal Gearmotor 25Dx56L mm MP 12v with 48 CPR Encoer(popolu.com)
  - HC-SR04 ultrasonic sensor
  
NOTE: sonar unit is cm (for accuracy reason)
      wheel and ecoder unit is in inches. 
      to use this lib alone. 

      #define botConst1 or #define botConst2
      #include<botConst.h> 

*/

#ifndef Driving_h
#define Driving_h

#include <Arduino.h>
#include "SabertoothSimplified.h"
#include "NewPing.h"
#include <botConst.h>

#include "I2Cdev.h"
#include "MPU6050.h"


/*********************************Ultrasonic Sensor constant*********************/
#define SONAR_NUM     5 // Number or sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).
#define BACK_TO_LEFT_SONAR 16.5

extern int SONAR_OFFSET[SONAR_NUM]; 



//UNCOMMENT this block below if decide not to use #include<botConst.h>
/***********************************************************************/
/**
//Driving.h const
#define WHEELDIAMETER 2.99 //2017 robot wheel 

adjust this value until the 2 encoder run at the same rate.
in order this lib to work as expected. 
 
#define R_MOTOR_MAX 98
#define L_MOTOR_MAX 104
#define L_TARGET_DIST_OFFSET 0.1
#define R_TARGET_DIST_OFFSET 0

#define L_TARGET_ANGLE_OFFSET 5
#define R_TARGET_ANGLE_OFFSET 4
//define controller constant
#define K1 0.438    //LEFT controller constant
#define K2 0.45743  //RIGHT controller constant
#define V 0.3//1.3      //speed controller constant
#define I 0.7   //integral controller constant
*/


/********************************motor driver***************************/
//define constant
// #define WHEELDIAMETER 2.7345 // metal wheel

#define ENCODERREVOLUTION 227.1 * 48
#define INCH2C ENCODERREVOLUTION / ( WHEELDIAMETER * 3.141592653589 * 2 * 2 )
#define COUNTER2INCHE ( WHEELDIAMETER * 3.14159265359 * 2 * 2) / ENCODERREVOLUTION
#define WHEELWIDTH 10.875
#define CM_TO_INCH 2.54

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
extern Vector accel;
extern Vector gyro;

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

/**********************************************************************/


//public functions--------------------------------------------------------
void dinit (void);		//default initiator, needed before using the functions from this library

void driveto( float distance );

/***
 * This function Drive the bot to a certain distance . 
 * by retrieve feedback from sonar and encoder to ensure that the bot move 
 * to a desire dist.
 * param dist( Dist you want the bot to travel) 
 * param sonarUse( toggle what sonnar to use for feedback check) 
 * sonarUse: 'f'(use front Sonar),
 *       'b'(use back Sonar), 
 *       '2'(use both Sonar),  
 * 
 * param straight(bool) (true - use checkParallel before get reading)
 *             false - don't straighten the bot. 
 * return
 */
void drivetoSonarFeedback(float dist, char sonarUse, bool straight);

void steer(int16_t toAngle );
/**
 * This function drive the robot to make sure 
 * that it stand paralel with the drive way.
 * float dispGoal: displacement goal 
 * so it can calulate how much it turn to reach 
 * the goal and be parallel to the goal.  
 */ 
void goParallel(float dispGoal,int leftDist, int rightDist);

void checkTilParallel();
/**
 * This method check if the mobile platform is parralel 
 * with the wall. 
 * if no, then approximate the angle that it is offset
 * by retrieving data from left and back sonar
 * accuracy: +- > 3.4 degree
 * return turnAngle( the degree that bot will turn for error checking)
 * param None
 */
int checkParallel();


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


/****************************sonar*****************************/
/**
 * This function compare the 2 dist retrieve by sonar. 
 * in this configure. 
 * sonar[0]: left sonar
 * sonar[1]: right sonar
 * sonar[2]: back sonar
 * return (positive) #: right > left 
 * return (negative) #: left > right 
 */
int sonarDistComparator();
int getSonarLeft();
int getSonarRight();
int getSonarLeftBack();

/**
 * This sonar require to be accurate in order for everything to work 
 * 
 * This function use api call provide by newPing lib 
 * then calibrate the reading using a linear regression model.
 * look at huskerBot 2018 src code folder to see the csv file with the model 
 * offset: -2
 * at the end compile will trunkate float to return int.
 */
int getSonarFront();

/**
 * 
 */
int getSonarBack();

void setSonarOffset(int *sonar);
void printAccuracySonar();


/********unuse*/
void imuDebug();
#endif
