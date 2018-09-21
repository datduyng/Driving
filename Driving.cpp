/*
Author:	CheeTown Liew
Version of June 10, 2017

This algorithm is designed to drive the robot and run with precise displacement
using motor encoder, the algorithm includes simple proportion controller and
displacement-encoder count convertor, speed comparator and path tracking.

Micro-controller Board: Arduino Mega
Motor Driver: Sabertooth Motor Driver


|						  |
|						  |
|						  |
|		|----------|	  |
|		|		   |	  |
|		|		   |	  |
|		|		   |	  |
|		|		   |	  |
|		|----------|	  |
|						  |

*/

#include "Driving.h"
#include <math.h>
//even: trigger
//odd: echo. 
SabertoothSimplified motordriver(Serial3);
MPU6050 mpu;
NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(22, 23, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. .// left 
  NewPing(24, 25, MAX_DISTANCE), // right
  NewPing(26, 27, MAX_DISTANCE), // back left 
  NewPing(28, 29, MAX_DISTANCE), // front sonar
  NewPing(30, 31, MAX_DISTANCE)  // back sonar
};

int SONAR_OFFSET[SONAR_NUM] ; 



//declare global & local coordinates
float global_x = 0.0;
float global_y = 0.0;
float local_x = 0.0;
float local_y = 0.0;
int16_t global_orientation = 0;

 float global_IMU_x = 0;
 float global_IMU_y = 0.0;
 float local_IMU_x = 0.0;
 float local_IMU_y=0.0 ;
 int16_t global_IMU_orientation=0.0;
 int16_t local_IMU_orientation=0.0 ;
 long ECLT =0;
 long ECRT=0;
 long left_targetcount1=0;
 long right_targetcount1=0;

Vector accel;
Vector gyro;

//declare encoder counter
volatile int64_t ECL = 0;    //encoder counter - LEFT
volatile int64_t ECLM = -1;  //encoder counter - LEFT MEMORY
volatile int64_t ECR = 0;    //encoder counter - RIGHT
volatile int64_t ECRM = -1;  //encoder counter - RIGHT MEMORY

//declare variable
bool debug1 = false;

void debug ( bool toggle ){
	debug1 = toggle;
}

//Default initiattion to assign I/O pins
void dinit (){
	//For Channel A
	attachInterrupt(digitalPinToInterrupt(ECLA), counter1, RISING); //LEFT encoder channel A trigger
	attachInterrupt(digitalPinToInterrupt(ECRA), counter2, RISING); //RIGHT encoder channel A trigger

	//For Channel B
	pinMode(ECLB, INPUT);   //LEFT encoder channel B as input
	digitalWrite(ECLB, LOW);  //Pull down LEFT channel B
	pinMode(ECRB, INPUT);   //RIGHT encoder channel B as input
	digitalWrite(ECRB, LOW);  //Pull down RIGHT channel B

	// //Encoder's Vcc
	pinMode(42, OUTPUT);
	pinMode(43, OUTPUT);
	pinMode(44, OUTPUT);
	pinMode(45, OUTPUT);
	digitalWrite(42, HIGH);
	digitalWrite(44, HIGH);
	digitalWrite(43, LOW);
	digitalWrite(45, LOW);

	/*
	pinMode(EPOWERL, OUTPUT);
	digitalWrite(EPOWERL, HIGH);
	pinMode(EPOWERR, OUTPUT);
	digitalWrite(EPOWERR, HIGH);
	*/


	// Serial.begin(9600);
	Serial3.begin(19200);
	
}

//Initiator with customize I/O pins, the argument must be pointer to an array with
//the format of { Left_Channel_A, Right_Channel_A, Left_Channel_B, Right_Channel_B };
/* void dinit ( uint8_t& pins ){
	attachInterrupt( digitalPinToInterrupt( *pins ), counter1, RISING);
	attachInterrupt( digitalPinToInterrupt( *(pins+1) ), counter2, RISING);

	pinMode( *(pins+2), INPUT);   //LEFT encoder channel B as input
	digitalWrite(*(pins+2), LOW);  //Pull down LEFT channel B
	pinMode( *(pins+3), INPUT);   //RIGHT encoder channel B as input
	digitalWrite( *(pins+4), LOW);  //Pull down RIGHT channel B

	//Encoder's Vcc
	pinMode(EPOWERL, OUTPUT);
	digitalWrite(EPOWERL, HIGH);
	pinMode(EPOWERR, OUTPUT);
	digitalWrite(EPOWERR, HIGH);
} */



//Left Encoder Counter
void static counter1() {
	ECLM = ECL;
	if (digitalRead(ECLB) == HIGH) {ECL++;}
	else {ECL--;}
}

//Right Encoder Counter
void static counter2() {
	ECRM = ECR;
	if (digitalRead(ECRB) == HIGH) {ECR++;}
	else {ECR--;}
}

int64_t D2C ( float distance ) {
	return ((double) distance * (double) INCH2C) ;
}

float C2D (int64_t EncoderCount ){
	return ((double) EncoderCount * (double) COUNTER2INCHE);
}

int16_t comparator( int64_t L, int64_t R ){
	// return abs( ECL - L ) - abs( ECR - R );
	return abs(L) - abs(R);
}

int16_t comparator2( int64_t L, int64_t R ){
	return abs( ECL - L ) - abs( ECR - R );
}

int16_t driving1 = 0;
int16_t driving2 = 0;
int16_t na = 0;

int64_t R2C ( int16_t angle ){
	return D2C ( ( (float) angle / 360.0 * (float) WHEELWIDTH * M_PI ) );
}


int16_t R2D (float radian){
	return radian * (180.0 / M_PI );
}

int16_t C2R ( int64_t encoderCount ){
	return C2D ( encoderCount ) / ( (float) WHEELWIDTH * M_PI ) * 360;
}

void steer ( int16_t toAngle ){
	int LEFT_OFFSET = 0;
	int RIGHT_OFFSET = 0;
	ECL = 0;
	ECR = 0;

	if(toAngle > 0){// if turn right
		LEFT_OFFSET = L_TARGET_ANGLE_OFFSET;
		RIGHT_OFFSET = R_TARGET_ANGLE_OFFSET;
	}else{// if turn left
		LEFT_OFFSET = -L_TARGET_ANGLE_OFFSET;
		RIGHT_OFFSET = -R_TARGET_ANGLE_OFFSET;
	}
	const int64_t left_targetcount = ECL + R2C(toAngle+LEFT_OFFSET);
	const int64_t right_targetcount = ECR - R2C(toAngle+RIGHT_OFFSET);
	/*
	Serial.println((int32_t)R2C(toAngle));
	Serial.println((int32_t)left_targetcount);
	Serial.println((int32_t)right_targetcount);
	*/
	driving1 = 0;// left drive
	driving2 = 0;
	na = 0;
	int8_t pause = 0;
	uint64_t timestamp = 0;
	int ECLO = -1;
	int ECRO = -1;
	const int ECLI = ECL;
	const int ECRI = ECR;
	int counter = 0; 
  	//Serial.print("left_targetcount");Serial.println((int32_t)left_targetcount);
  	//Serial.print("right_targetcount");Serial.println((int32_t)right_targetcount);

	while ( true ) {
		driving1 = constrain( (double) K1 * (left_targetcount - ECL), -L_MOTOR_MAX, L_MOTOR_MAX);
		driving2 = constrain( (double) K2 * (right_targetcount - ECR), -R_MOTOR_MAX, R_MOTOR_MAX);

/*
the comparison value of the encoder count difference per iteration
positive mean left motor is faster than the right motor and vise versa
*/

		int16_t v = comparator2( ECLO, ECRO );
		ECLO = ECL;
		ECRO = ECR;
		if( (v*(float) V) > 30.0 ){} //Serial.println("Warning! There is a large speed differential.");
		else v *= (float) V;
		// Serial.print("ECL:");
		// Serial.print( (int32_t) ECL);
		// Serial.print("\tECR:");
		// Serial.print( (int32_t) ECR);
		if(toAngle > 0 ){

			if( v > 0 ){
				if (driving1 > 0){
					driving1 -= v* (float) V;
				}else{
					driving1 += v* (float) V;
				}
			} else {
				if (driving2 > 0){
					driving2 -= v* (float) V;
				}else{
					driving1 += v* (float) V;
				}
			}
		}else {
			if( v > 0 ){
				if (driving1 > 0){
					driving1 -= v* (float) V;
				}else{
					driving1 += v* (float) V;
				}
			} else {
				if (driving2 > 0){
					driving2 -= v* (float) V;
				}else{
					driving1 += v* (float) V;
				}
			}

		}
		// Serial.print("\tECLO:");
		// Serial.print( (int32_t) ECLO);
		// Serial.print("\tECRO:");
		// Serial.print( (int32_t) ECRO);
		// Serial.print("\tV1:");
		// Serial.print(driving1);
		// Serial.print("\tV2:");
		// Serial.print(driving2);
		// Serial.println("");
		int16_t integral = comparator2( ECLI, ECRI );

		if (toAngle> 0){
			driving1 = constrain( driving1, 0, L_MOTOR_MAX);
			driving2 = constrain( driving2, -R_MOTOR_MAX, 0);
		}else{
			driving1 = constrain(driving1, -L_MOTOR_MAX, 0);
			driving2 = constrain(driving2, 0, R_MOTOR_MAX);
		}
		if (counter < 10){
			driving1 -= 20;
			driving2 -= 20;
		}
		counter ++;


		motordriver.motor(1, driving1);
		motordriver.motor(2, driving2);

		// if(ECL == left_targetcount || ECR == right_targetcount){
		// 	break;
		// }
		//if the motor(s) stall for 0.1s, quit the iteration
		if ( ECL == ECLO || ECR == ECRO ) {
			if ( pause == 0 ) {
				timestamp = millis();
				pause = 1;
			} else {
				if ( abs(millis() - timestamp) > 200 ) {
					pause = 0;
					//Serial.println("break");
					break;
				}
			}
		}


		//if (debug == true)debugMode();
		/*
		Serial.print( (int32_t) ECL);
		Serial.print("\t");
		Serial.print( (int32_t) ECR);
		Serial.print("\t");
		Serial.print(v);
		Serial.print("\t");
		Serial.print(integral);
		Serial.print("\t");
		Serial.print(driving1);
		Serial.print("\t");
		Serial.print(driving2);
		Serial.print("\n");
		*/
	}

	motordriver.motor(1, 0);
	motordriver.motor(2, 0);


	global_orientation +=  ( ( C2R ( (int64_t) ECL - (int64_t) ECLI ) -  C2R( ( int64_t) ECR - (int64_t) ECRI ))/2 );
	//global_orientation += toAngle;
	//global_orientation = constrain(global_orientation, -180, 180);

	//Serial.println(global_orientation);
	ECL = 0;
	ECR = 0;
}

void driveto( float distance ) {
	ECL = 0;
	ECR = 0;
	const int64_t left_targetcount = D2C( distance + L_TARGET_DIST_OFFSET );
	const int64_t right_targetcount = D2C( distance + R_TARGET_DIST_OFFSET );

	// deposite to global_x displacement.
	global_y += distance; 
	driving1 = 0;
	driving2 = 0;
	na = 0;
	int8_t pause = 0;
	uint64_t timestamp = 0;
	int ECLO = -1;
	int ECRO = -1;
	int left_encoder_offset= 0;
	int right_encoder_offset = 0;
	const int ECLI = ECL;
	const int ECRI = ECR;
	int mode = 0; 
	// Serial.begin(9600);
  	// Serial.print("left_targetcount");Serial.println((int32_t)left_targetcount);
  	// Serial.print("right_targetcount");Serial.println((int32_t)right_targetcount);

	// if(abs(abs(ECL) - abs(ECR)) > 100){
	// 	Serial.println("used mode 2");
	// 	mode = 2; 
	// }else{
	// 	Serial.println("used mode 1");
 //  		mode = 1; 
 //  	}
  	mode = 1;
	while ( true ) {
		// debugMode();
		/****************DEBUG*****************/
		driving1 = constrain( (double) K1 * (left_targetcount - ECL), -L_MOTOR_MAX, L_MOTOR_MAX);
		driving2 = constrain( (double) K2 * (right_targetcount - ECR), -R_MOTOR_MAX, R_MOTOR_MAX);
	
/*	
the comparison value of the encoder count difference per iteration
positive value indicates left motor is faster than the right motor and vise versa
*/		


		int16_t v;
		if(mode == 1){
			v = comparator( ECLO, ECRO);
		}else if(mode == 2){
			v = comparator2( ECLO, ECRO);
		}

		ECLO = ECL;
		ECRO = ECR;
		
		// Serial.print("ECL:");
		// Serial.print( (int32_t) ECL);
		// Serial.print("\tECR:");
		// Serial.print( (int32_t) ECR);
		// Serial.print("\tdiff(v):");Serial.print(v);
		// Serial.print("\tV:");Serial.print(v*(float) V);

		if( (v*(float) V) > 30.0 ){} //Serial.println("Warning! There is a large speed differential.");
		else v *= (float) V;
		if(distance > 0 ){
			
			if( v > 0 ){
				if (driving1 > 0){
					driving1 -= v* (float) V;
				}else{
					driving1 += v* (float) V;
				}				
			} else {
				if (driving2 > 0){
					driving2 -= v* (float) V;
				}else{
					driving1 += v* (float) V;
				}
			}
		}else {
			if( v > 0 ){
				if (driving1 > 0){
					driving1 -= v* (float) V;
				}else{
					driving1 += v* (float) V;
				}				
			} else {
				if (driving2 > 0){
					driving2 -= v* (float) V;
				}else{
					driving1 += v* (float) V;
				}
			}
			
		}
		
		int16_t integral;
		if(mode == 1){
			integral = comparator( ECLI, ECRI);
		}else if(mode == 2){
			integral = comparator2( ECLI, ECRI);
		}
		if( integral*  (float) I > 20 ){}  //Serial.println("Warning! There is a large integral differences.");
		if(distance > 0 ){
			
			if( integral > 0 ){
				if (driving1 > 0){
					driving1 -= integral* (float) I;
				}else{
					driving2 += integral* (float) I;
				}				
			} else {
				if (driving2 > 0){
					driving2 -= integral* (float) I;
				}else{
					driving1 += integral* (float) I;
				}
			}
		}else {
			if( integral > 0 ){
				if (driving1 < 0){
					driving1 -= integral* (float) I;
				}else{
					driving2 += integral* (float) I;
				}				
			} else {
				if (driving2 < 0){
					driving2 += integral* (float) I;
				}else{
					driving1 -= integral* (float) I;
				}
			}
			
		}
		// Serial.print("\tECLO:");
		// Serial.print( (int32_t) ECLO);
		// Serial.print("\tECRO:");
		// Serial.print( (int32_t) ECRO);
		// Serial.print("\tV1:");
		// Serial.print(driving1);
		// Serial.print("\tV2:");
		// Serial.print(driving2);
		// Serial.println("");
		if (distance > 0){
			//forward motion
			driving1 = constrain( driving1, 0, L_MOTOR_MAX);
			driving2 = constrain( driving2, 0, R_MOTOR_MAX);
		}else{
			//backward motion
			driving1 = constrain(driving1, -L_MOTOR_MAX, 0);
			driving2 = constrain(driving2, -R_MOTOR_MAX, 0);
		}
		
		
		

		motordriver.motor(1, driving1);
		motordriver.motor(2, driving2);

		// if((abs(ECL)-abs(left_targetcount)) <= 1 || (abs(ECL)-abs(left_targetcount)) <= 1){
		// 	break;
		// }

		//if the motor(s) stall for 0.1s, quit the iteration 
		if ( ECL == ECLO || ECR == ECRO ) {
			if ( pause == 0 ) {
				timestamp = millis();
				pause = 1;
			} else {
				if ( abs(millis() - timestamp) > 300 ) {
					pause = 0;
					break;
				}
			}
		}

		//if (debug == true)debugMode();
		/*
		Serial.print( (int32_t) ECL);
		Serial.print("\t");
		Serial.print( (int32_t) ECR);
		Serial.print("\t");
		Serial.print(v);
		Serial.print("\t");
		Serial.print(integral);
		Serial.print("\t");
		Serial.print(driving1);
		Serial.print("\t");
		Serial.print(driving2);
		Serial.print("\n");
		*/
	}

	motordriver.motor(1, 0);
	motordriver.motor(2, 0);

	global_x += ( C2D( ECL - ECLI) + C2D (ECR - ECRI) )/2 * cos(global_orientation);
	global_y += ( C2D( ECL - ECLI) + C2D (ECR - ECRI) )/2 * sin(global_orientation);
	//global_x = C2D (( ECL + ECR )/2) ;
	// Serial.println("done moving straight");
	
	ECL = 0;
	ECR = 0;

}

void driveToMpu(float dist){
  // time is calculated in microS
  long currentTime = 0;
  long previousTime =0;
  float theta = 0;
  float error = 0, current = 0;
  float target = 0; // target is alway 0; robot need to have 0 yaw.
  float p = 0,i = 0,d = 0, pid = 0;
  int8_t drivingR, drivingL = 0;
	float dispX=0;
	float orientationDisp =0;
	double speed = 90;
  // elapsed time is in ms
  float elapsedTime = 0;

  local_IMU_x = 0;
  local_IMU_y = 0;

	target=dist;
  while(true){

    // start timer
    previousTime = micros();

    // update reading from mpu
    accel = mpu.readNormalizeAccel();
	gyro =  mpu.readNormalizeGyro();

    theta = R2D((float)gyro.YAxis* elapsedTime) + 1.00;
    p = theta * kp ;
    i = i + theta*ki;

    pid = (p + i);
		// speed = (-target/(-target)) * speed ;
		// Serial.print("===pid:");Serial.print(pid);
		// Serial.print("\t===time: ");Serial.print(elapsedTime);
		if(pid > 0){
			drivingR = constrain((speed+pid), -R_MOTOR_MAX, R_MOTOR_MAX );// -MAX = MIn
	    drivingL = constrain(speed, -L_MOTOR_MAX, L_MOTOR_MAX );// =MAX = min
		}else if(pid < 0){
			drivingR = constrain(speed, -R_MOTOR_MAX, R_MOTOR_MAX );// -MAX = MIn
	    drivingL = constrain(speed+pid, -L_MOTOR_MAX, L_MOTOR_MAX );// =MAX = min
		}

		// Serial.print("\t|DrivingR:");Serial.print(drivingR);
		// Serial.print("\tDrivingL:");Serial.print(drivingL);
		// Serial.print("\ttheta:");Serial.print(theta,6);
		// Serial.print("\tspeed:");Serial.print(speed);
		// Serial.print("\ttarget");Serial.print(target);
		// Serial.print("\tdisp:");Serial.println(dispX,6);

		imuDebug();
    // check to see if run backward or forward
    if(dist > 0){
      drivingR = constrain( drivingR, 0, R_MOTOR_MAX);
      drivingL = constrain( drivingL, 0, L_MOTOR_MAX);
    }else{
      //backward motion
			drivingR = constrain(drivingR, -R_MOTOR_MAX, 0);
			drivingL = constrain(drivingL, -L_MOTOR_MAX, 0);
		}

    motordriver.motor(1, drivingR);
    motordriver.motor(2, drivingL);

    // end the timer and calculate the elapsed time
  	currentTime = micros();
    elapsedTime = (float)(currentTime - previousTime)/1000000.0;// convert to s

		dispX = (0.5) * ((float)accel.XAxis) * (elapsedTime * elapsedTime);
		orientationDisp = (0.5) * ((float)gyro.YAxis+0.20) * (elapsedTime * elapsedTime);

		global_IMU_x += (float)dispX ;
		//global_IMU_y += (float)disp;

		target -= ((float)dispX);
		// stop the drive when the bot has reached designated dist.
		if(target < 0.0){
			break;
		}
  }// end true
	// Serial.println("end");
  motordriver.motor(1, 0);
  motordriver.motor(2, 0);
}

bool turnWImu(int degree){

	//error checking
	if(degree == 0){
		Serial.println("Turn 0 degree ? ");
		return false;
	}

	long previousTime = 0;
	long currentTime = 0;
	float elapsedTime = 0;
	float drivingR = 0 , drivingL = 0;

	local_IMU_orientation = 0;
	uint16_t drivingConstrain = abs(degree);
	while(true){

		// start timer
		previousTime = micros();

			// take proportion when turning to prevent overshooting the designated reaching goal
		drivingConstrain -=  degree - abs(local_IMU_orientation - degree);

		// update reading from mpu
    accel = mpu.readNormalizeAccel();
		gyro =  mpu.readNormalizeGyro();

		double theta = (float)gyro.YAxis* elapsedTime;
		global_IMU_orientation += R2D(theta); // convert to degree

		local_IMU_orientation = global_IMU_orientation;

		if(degree > 0){// rotate CW
			drivingR = constrain( drivingConstrain, -R_MOTOR_MAX,0 );
			drivingL = constrain( drivingConstrain, 0, L_MOTOR_MAX);
		}else if(degree < 0){
			drivingL = constrain( drivingConstrain, -R_MOTOR_MAX,0 );
			drivingR = constrain( drivingConstrain, 0, L_MOTOR_MAX);
		}

		if(local_IMU_orientation == degree){ // reached the goal.
			break;
		}

		// end the timer and calculate the elapsed time
		currentTime = micros();
		// convert to ms
		elapsedTime = (float)(currentTime - previousTime)/1000.0;

	}// end while(true)
	motordriver.motor(1, 0);
  motordriver.motor(2, 0);
	return true;
}


bool imuInit(void){
	if(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_16G))
	  {
	    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
	    delay(500);
	  }

	mpu.setAccelPowerOnDelay(MPU6050_DELAY_3MS);

	mpu.setIntFreeFallEnabled(false);
	mpu.setIntZeroMotionEnabled(false);
	mpu.setIntMotionEnabled(false);

	mpu.setDHPFMode(MPU6050_DHPF_5HZ);

	mpu.setMotionDetectionThreshold(2);
	mpu.setMotionDetectionDuration(5);

	mpu.setZeroMotionDetectionThreshold(4);
	mpu.setZeroMotionDetectionDuration(2);

}

void imuDebug(void){
	// update reading from mpu
	// Serial.begin(9600);
	accel = mpu.readNormalizeAccel();
	gyro =  mpu.readNormalizeGyro();


	Serial.print("x");Serial.print(accel.XAxis);
	Serial.print("\ty");Serial.print(accel.YAxis);
	Serial.print("\tz");Serial.print(accel.ZAxis);
	Serial.print("\t|||x");Serial.print(gyro.XAxis);
	Serial.print("\ty");Serial.print(gyro.YAxis);
	Serial.print("\tz");Serial.println(gyro.ZAxis);
	//delay(1000);
	delay(100);
}


void debugMode (){
	//Serial.begin(9600);
	Serial.print(na);
	Serial.print("\t");
	Serial.print("ECL:");
	Serial.print( (int32_t) ECL);
	Serial.print("\tECR:");
	Serial.print( (int32_t) ECR);
	Serial.print("\tV1:");
	Serial.print(driving1);
	Serial.print("\tV2:");
	Serial.print(driving2);
	Serial.print("\tleft_target");Serial.print(left_targetcount1);
	Serial.print("\tright_target");Serial.print(right_targetcount1);
	Serial.println("");
}
void driveToPID(float dist){
	int driveR = 100*(dist/fabs(dist));
	int driveL = 100*(dist/fabs(dist));
	left_targetcount1 = (long)ECL+D2C( dist);
	right_targetcount1 = (long)ECR+D2C( dist);
	ECLT = 0;
	ECRT = 0;
	float error =0.0;
	//Serial.begin(9600);
	while(true){
		//debugMode();
		Serial.print("ECL: ");Serial.print( (int32_t) ECL);
		Serial.print("\tECR: ");Serial.print( (int32_t) ECR);
		ECLT = (long)ECL;
		ECRT = (long) ECR;
		error = ECLT - ECRT;
		Serial.print("\terror:");Serial.print(error);
		driveR += round(error / kp);
		Serial.print("\tdriveL: ");Serial.print(driveL);
		Serial.print("\t#driveR: ");Serial.print(driveR);
		Serial.println("");
		driveL = constrain( (double) driveL, -127, 127);
		driveR = constrain( (double) driveR, -127, 127);

		if (dist > 0){
			//forward motion
			driveL = constrain( driveL, 0, 127);
			driveR = constrain( driveR, 0, 127);
		}else{
			//backward motion
			driveL = constrain(driveL, -127, 0);
			driveR = constrain(driveR, -127, 0);
		}

		motordriver.motor(1, driveL);
		motordriver.motor(2, driveR);


		if(left_targetcount1-ECL <= 0){
			motordriver.motor(1, 0);
		}
		if(right_targetcount1-ECR <= 0){
			motordriver.motor(2, 0);
		}
		if(left_targetcount1-ECL <= 0 && right_targetcount1-ECR <= 0){
			break;
		}




		// restart encoder to have fresh error value. 
		ECLT = 0;
		ECRT = 0; 
	}
	motordriver.motor(1, 0);
	motordriver.motor(2, 0);

}

void goParallel(float dispGoal,int leftDist, int rightDist){
	float paralelOffsetInInches = ((float)(rightDist - leftDist)/CM_TO_INCH)/2;
	// Serial.println("=====++++++++++++++======");
	// Serial.print("parallelOffset:");Serial.print(paralelOffsetInInches);
	// Serial.print("dispGoal");Serial.print(dispGoal);
	if(abs(paralelOffsetInInches) > 7.0){// undetermine case. 
		driveto(dispGoal);
		return;
	}
	float turnInRad = paralelOffsetInInches/dispGoal;
	// Serial.print("turnInRad:");Serial.print(turnInRad);
	// Serial.println("==========");
	float magnitude = sqrt((paralelOffsetInInches*paralelOffsetInInches)+(dispGoal*dispGoal));
	steer(R2D(turnInRad));
	delay(100);
	driveto(magnitude);
	delay(100);
	steer(-R2D(turnInRad)); // steer back to be parallel.
	// checkParallel(); 

}
int checkParallel(){
	int left = getSonarLeft();
	int back = getSonarLeftBack();

	//error check if there is malfunction in sonar 
	// or the bot is straight already. 
	if(abs(back-left) > 10 || left == 0 || back == 0){
		return 0;
	}
	int turnAngle = R2D(asin((float)(back-left)/BACK_TO_LEFT_SONAR));
	steer(turnAngle);
	return turnAngle;
}	

void checkTilParallel(){
	int left = getSonarLeft();
	int back = getSonarLeftBack();
	int counter = 0; 

	//error check if there is malfunction in sonar 
	// or the bot is straight already. 
	if(abs(back-left) > 10  || left == 0 || back == 0){
		return 0;
	}
	while(abs(left-back) >= 2){
		left = getSonarLeft();
		back = getSonarLeftBack();
		if(left > back) steer(-1);
		else if(left < back) steer(1);
		delay(200);
		counter ++;
		if(counter == 4){
			break;
		}
	}

}	

int sonarDistComparator(){
	int  leftDistance = getSonarLeft();
	int rightDistance = getSonarRight();
	if(rightDistance == 0) return 1111; // return positive number/malfunction or out of range 
	if(leftDistance == 0) return -1111; // return positive number/malfunction or out of range 
	
	//divide by 2 to get offset from midpoint. 
	return (rightDistance-leftDistance)/2; // with this year robot configuration #1 is the right. #0 is the left sonar 
	//if return (-)# left > right 
	// if retun (+)# right> left   
}

int getSonarLeft(){
	int result = sonar[0].ping_cm() + SONAR_OFFSET[0];


	if(result < 0) return 0;
	else return result;
}

int getSonarRight(){
	int result = sonar[1].ping_cm() + SONAR_OFFSET[1];
	if(result < 0) return 0;
	else return result;
}

int getSonarLeftBack(){
	int result = sonar[2].ping_cm() + SONAR_OFFSET[2];
	if(result < 0) return 0;
	else return result;
}


int getSonarFront(){
	int result = sonar[3].ping_cm() + SONAR_OFFSET[3];
	
	if(result >= 7 && result <= 16){//y=x+1 
		result = result + 1 ;
	}else if(result >= 22 && result <= 32){//y =.82+2.6995
		int bias = 0;// check excel sheet.
		result = round((0.82*(float)result) + 2.6995 - bias);

		//after using model collected offset 
		if(result >= 32 && result <= 34){//y= .75x + 5.75 
			result = result * 0.75 + 5.75;
		}
	}else if(result >= 33 && result <= 37){//y=x-1
		result = result - 1;
	}

	// check if <0 then sonar has malfunction.
	if(result < 0) return 0;
	else return result;
}

int getSonarBack(){
	int result = sonar[4].ping_cm() + SONAR_OFFSET[4];
	if(result < 0) return 0;
	if(result >= 17) return result - 1;
	else result;
}

void setSonarOffset(int *offset){

	for(int i = 0;i < SONAR_NUM; i++){
		SONAR_OFFSET[i] = offset[i];
	}
	//2,-2,0 for the 1st robot 
}




void drivetoSonarFeedback(float dist, char sonarUse, bool straight){
	int beforeFrontDist ;
	int beforeBackDist;
	int afterFrontDist;
	int afterBackDist; 

	//make sure the bot is parallel.
	if(straight == true) checkParallel();

	//see which sonar to use for feedback.
	if(sonarUse == 'f') beforeFrontDist = getSonarFront();
	else if(sonarUse == 'b') beforeBackDist = getSonarBack();

	if(straight == true) checkParallel();
	driveto(dist);

	//see which sonar to use for feedback.
	if(sonarUse == 'f') afterFrontDist = getSonarFront();
	else if(sonarUse == 'b') afterBackDist = getSonarBack();

	//malfunction sonar reading.
	if(beforeFrontDist == 0 || afterFrontDist==0){
		return;
	}
	//get actual disp by sonar
	float actualFrontDisp =(float)(beforeFrontDist- afterFrontDist)/CM_TO_INCH;
	float actualBackDisp =(float)(afterBackDist - beforeBackDist)/CM_TO_INCH; 

	if(sonarUse == 'f') driveto(actualFrontDisp-dist);
	
	else if(sonarUse == 'b') {
		driveto(dist - actualBackDisp);
		Serial.println("use");Serial.print(dist-actualBackDisp);
		Serial.println("");
	}

}	

