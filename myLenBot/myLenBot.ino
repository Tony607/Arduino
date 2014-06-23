// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"
#include "SPI.h"
#include "Kalman.h"
#include "sonarCaptureT1.h"
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include <TimerThree.h>
#include <AccelStepper.h>

#define pinStep1 6
#define pinDirection1 7
#define pinStep2 8
#define pinDirection2 9
AccelStepper stepper(AccelStepper::DRIVER, pinStep1, pinDirection1);
AccelStepper stepper2(AccelStepper::DRIVER, pinStep2, pinDirection2);

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax=0, ay=0, az=0;
int16_t gx=0, gy=0, gz=0;
/* IMU Data */
double accX=0.0, accY=0.0, accZ=0.0;
double gyroX=0.0, gyroY=0.0;
char sendcnt = 0;
double kalAngleX=0.0, kalAngleY=0.0; // Calculated angle using a Kalman filter
volatile int motorSpeed = 0; 
//for ultrasonic sensor
uint32_t timer=0;

#define LED_PIN 13
bool blinkState = false;
bool testresult = false;
void getIMUdouble(){
	accX = (double)ax;
	accY = (double)ay;
	accZ = (double)az;
	gyroX= (double)gx;
	gyroY= (double)gy;
}
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial1.begin(115200);
	//while(!Serial){
	//	;
	//}
    initSonarCaptureT1();
    // initialize device
    //Serial1.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    //Serial1.println("Testing device connections...");
    testresult = accelgyro.testConnection();
    //Serial1.println(testresult ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);

    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);                                                                                                                                                                                                      
	getIMUdouble();
	double roll  = atan2(accY, accZ) * RAD_TO_DEG;
	double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	kalmanX.setAngle(roll); // Set starting angle
	kalmanY.setAngle(pitch);
	timer = micros();

   stepper.setMaxSpeed(2000);
   stepper.setSpeed(500);	
   stepper2.setMaxSpeed(2000);
   stepper2.setSpeed(1000);
   Timer3.initialize(100);//100us,10kHz
   Timer3.attachInterrupt(rampSpeed);
}
void rampSpeed(void){
   stepper.setSpeed(-motorSpeed);
   stepper2.setSpeed(motorSpeed);
   stepper.runSpeed();
   stepper2.runSpeed();
}
void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	getIMUdouble();
	double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
	timer = micros();
	double roll  = atan2(accY, accZ) * RAD_TO_DEG;
	double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	double gyroXrate = gyroX / 131.0; // Convert to deg/s
	double gyroYrate = gyroY / 131.0; // Convert to deg/s

	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
		kalmanX.setAngle(roll);
		kalAngleX = roll;
	} else{
		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
	}
	if (abs(kalAngleX) > 90){
		gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
	}
	kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    
if(testresult){
    motorSpeed = constrain((int)(kalAngleX*30), -2000, 2000);
    // display tab-separated accel/gyro x/y/z values
	if(sendcnt<10){
		sendcnt++;
	}else{
		sendcnt=0;
                trigPin();
		Serial1.print(kalAngleX);
		Serial1.print(F(":"));
		Serial1.print(kalAngleY);
		Serial1.print(F("#"));
		Serial1.print(motorSpeed);
		Serial1.println(F("*"));
		/*Serial1.print(gyroYrate);
		Serial1.print(F(":"));
		Serial1.print(pitch);
		Serial1.print(F(":"));
		Serial1.print(dt);
		Serial1.print(F("\r\n"));*/

	}
	delay(1);
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}else{
  Serial1.println("test fail");
  delay(500);
}
}
