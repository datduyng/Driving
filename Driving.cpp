/*
Author:	CheeTown Liew
Version of June 10, 2017

This algorithm is designed to drive the robot and run with precise displacement
using motor encoder, the algorithm includes simple proportion controller and
displacement-encoder count convertor, speed comparator and path tracking.

Micro-controller Board: Arduino Mega
Motor Driver: Sabertooth Motor Driver

*/

#include <Driving.h>
#include <math.h>

SabertoothSimplified motordriver(Serial3);
MPU6050 accelgyro;


//declare global & local coordinates
float global_x = 0.0;
float global_y = 0.0;
float local_x = 0.0;
float local_y = 0.0;
int16_t global_orientation = 0;

int16_t ax, ay, az;
int16_t gx, gy, gz;

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

	//Encoder's Vcc
	pinMode(24, OUTPUT);
	pinMode(25, OUTPUT);
	pinMode(26, OUTPUT);
	pinMode(27, OUTPUT);
	digitalWrite(24, HIGH);
	digitalWrite(25, HIGH);
	digitalWrite(26, LOW);
	digitalWrite(27, LOW);

	/*
	pinMode(EPOWERL, OUTPUT);
	digitalWrite(EPOWERL, HIGH);
	pinMode(EPOWERR, OUTPUT);
	digitalWrite(EPOWERR, HIGH);
	*/

	//Serial.begin(9600);

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
	const int64_t left_targetcount = ECL + R2C(toAngle + 2);
	const int64_t right_targetcount = ECR - R2C(toAngle + 2);
	/*
	Serial.println((int32_t)R2C(toAngle));
	Serial.println((int32_t)left_targetcount);
	Serial.println((int32_t)right_targetcount);
	*/
	driving1 = 0;
	driving2 = 0;
	na = 0;
	int8_t pause = 0;
	uint64_t timestamp = 0;
	int ECLO = -1;
	int ECRO = -1;
	const int ECLI = ECL;
	const int ECRI = ECR;

	while ( true ) {
		driving1 = constrain( (double) K1 * (left_targetcount - ECL), -113, 112);
		driving2 = constrain( (double) K2 * (right_targetcount - ECR), -127, 127);

/*
the comparison value of the encoder count difference per iteration
positive mean left motor is faster than the right motor and vise versa
*/

		int16_t v = comparator( ECLO, ECRO );
		if( (v*(float) V) > 30.0 ){} //Serial.println("Warning! There is a large speed differential.");
		else v *= (float) V;
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

		int16_t integral = comparator( ECLI, ECRI );
		// if( integral*  (float) I > 20 ) Serial.println("Warning! There is a large integral difference");
		// if(toAngle > 0 ){

			// if( integral > 0 ){
				// if (driving1 > 0){
					// driving1 -= integral* (float) I;
				// }else{
					// driving2 += integral* (float) I;
				// }
			// } else {
				// if (driving2 > 0){
					// driving2 -= integral* (float) I;
				// }else{
					// driving1 += integral* (float) I;
				// }
			// }
		// }else {
			// if( integral > 0 ){
				// if (driving1 < 0){
					// driving1 += integral* (float) I;
				// }else{
					// driving2 += integral* (float) I;
				// }
			// } else {
				// if (driving2 < 0){
					// driving2 -= integral* (float) I;
				// }else{
					// driving1 -= integral* (float) I;
				// }
			// }

		// }

		if (toAngle> 0){
			driving1 = constrain( driving1, 0, 112);
			driving2 = constrain( driving2, -127, 0);
		}else{
			driving1 = constrain(driving1, -113, 0);
			driving2 = constrain(driving2, 0, 127);
		}




		motordriver.motor(1, driving1);
		motordriver.motor(2, driving2);

		//if the motor(s) stall for 0.1s, quit the iteration
		if ( ECL == ECLO || ECR == ECRO ) {
			if ( pause == 0 ) {
				timestamp = millis();
				pause = 1;
			} else {
				if ( abs(millis() - timestamp) > 100 ) {
					pause = 0;
					break;
				}
			}
		}

		ECLO = ECL;
		ECRO = ECR;
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

}

void driveto( float distance ) {
	// time is calculated in microS
	// long currentTime = 0;
	// long previousTime =0;

	// elapsed time is in ms
	// float elapsedTime = 0;

	// global_IMU_x = 0;
	// global_IMU_y = 0;
	// global_IMU_z = 0;

	const int64_t left_targetcount = ECL+ D2C( distance + 0.15 );
	const int64_t right_targetcount = ECR + D2C( distance + 0.15 );
	driving1 = 0;
	driving2 = 0;
	na = 0;
	int8_t pause = 0;
	uint64_t timestamp = 0;
	int ECLO = -1;
	int ECRO = -1;
	const int ECLI = ECL;
	const int ECRI = ECR;

	while ( true ) {
		driving1 = constrain( (double) K1 * (left_targetcount - ECL), -113, 112);
		driving2 = constrain( (double) K2 * (right_targetcount - ECR), -127, 127);

		// start timer
		//previousTime = micros();


		// read accelerometer and gyro
		// bno.getEvent(&event);
		// imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
		// imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
		//
		// global_IMU_orientation = R2D (gyro.y * elapsedTime);
		// global_IMU_x += (1/2) * ((float)accel.x) * (elapsedTime * elapsedTime);
		// global_IMU_y += (1/2) * ((float)accel.y) * (elapsedTime * elapsedTime);

/*
the comparison value of the encoder count difference per iteration
positive value indicates left motor is faster than the right motor and vise versa
*/
		int16_t v = comparator( ECLO, ECRO );
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

		int16_t integral = comparator( ECLI, ECRI );

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


		if (distance > 0){
			//forward motion
			driving1 = constrain( driving1, 0, 112);
			driving2 = constrain( driving2, 0, 127);
		}else{
			//backward motion
			driving1 = constrain(driving1, -113, 0);
			driving2 = constrain(driving2, -127, 0);
		}




		motordriver.motor(1, driving1);
		motordriver.motor(2, driving2);

		//if the motor(s) stall for 0.1s, quit the iteration
		if ( ECL == ECLO || ECR == ECRO ) {
			if ( pause == 0 ) {
				timestamp = millis();
				pause = 1;
			} else {
				if ( abs(millis() - timestamp) > 100 ) {
					pause = 0;
					break;
				}
			}
		}

		ECLO = ECL;
		ECRO = ECR;
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

	// end the timer
	// and calculate the elapsed time
	//currentTime = micros();
  //elapsedTime = (currentTime - previousTime)/1000.0;
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

  // elapsed time is in ms
  float elapsedTime = 0;

  local_IMU_x = 0;
  local_IMU_y = 0;

  while(true){

    // start timer
    previousTime = micros();

    // update reading from mpu
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    theta = (float)gy * elapsedTime;
    p = theta * kp;
    i = i + theta*ki;

    pid = p + i;

    drivingR = constrain(pid, -R_MOTOR_MAX, R_MOTOR_MAX );// -MAX = MIn
    drivingL = constrain(pid, -L_MOTOR_MAX, L_MOTOR_MAX );// =MAX = min

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

    global_IMU_x += (1/2) * ((float)ax) * (elapsedTime * elapsedTime);
    global_IMU_y += (1/2) * ((float)ay) * (elapsedTime * elapsedTime);

		local_IMU_x = global_IMU_x;
		local_IMU_y = global_IMU_y;

		// stop the drive when the bot has reached designated dist.
		if(local_IMU_x == dist){
			break;
		}
  }// end true

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
		accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		double theta = (float)gy * elapsedTime;
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
  // initialize
  ax = 0, ay = 0, az = 0, gx = 0 , gy = 0, gz = 0;

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif

    accelgyro.initialize();
    return accelgyro.testConnection();
}




void debugMode (){
	Serial.begin(9600);
	Serial.print(na);
	Serial.print("\t");
	Serial.print("ECL\t");
	Serial.print( (int32_t) ECL);
	Serial.print("\tECR\t");
	Serial.print( (int32_t) ECR);
	Serial.print("\tV1\t");
	Serial.print(driving1);
	Serial.print("\tV2\t");
	Serial.println(driving2);
}
