/*
Small Scale Autonomous Boat (Capstone) Project
Name:		Autonomous_Bouat.ino
Created:	1/10/2016 2:16:56 PM
*/

// Program Header file includesS



/* ------------------------------------------------------ */

// Program Variable Assignment

#include <SoftwareSerial.h>
#include <Wire.h>
#include <AFMotor.h>
//#include <Servo.h>
#include <MeOrion.h>
#include <MeBluetooth.h>
#include <MeCompass.h>
#include <MeOrion.h>


const int threshold = 60; // distance from object we want things to happen
const double Tol = 15.0;	// Heading angle tolerance number

/* ------------------------------------------------------ */

// Ultrasonic Assignments
#define echoPin 13 // Echo Pin
#define trigPin 12 // Trigger Pin

const uint8_t address = 30; // I2C Address hex = 0x1 dec =30

long duration, distance; // Duration used to calculate distance

/* ------------------------------------------------------ */

// Compass assignment

//#define COMPASS_AVERAGING_4 
//#define COMPASS_MODE_CONTINUOUS 
//#define DEBUG_ENABLED  1
//#define COMPASS_RATE_75            (0x18)   // 75   (Hz)

double angle_number;
double angle_fwd;	// Starting angle
double angle_pTol; // Positive fwd tol
double angle_nTol;	// Negative fwd tol
double angle_A;	// Meas. 1 angle in deg
double angle_A_pTol;	// Meas. 1 + Tolerance
double angle_A_nTol;	// Meas. 1 - Tolerance

int16_t head_X, head_Y, head_Z;	//	Compass variable assignment

const uint8_t keyPin = 3;	//	Compass pin assignment variable
const uint8_t ledPin = 2;	//	Compass pin assignment variable

MeCompass Compass;
/* ------------------------------------------------------ */

// Bluetooth Assignments

uint8_t rx_pin = 0;
uint8_t tx_pin = 1;

unsigned char table[128] = { 0 };	//	Bluetooth table 

#define MeBluetooth(rx_pin, tx_pin, false);

MeBluetooth bluetooth;

/* ------------------------------------------------------ */

// Motor Assignments

const int SERVO = 9;

Servo myServo;	// Servo object
AF_DCMotor motor(4);	// Motor object

/* ------------------------------------------------------ */

void setup()
{
	// Serial Setup
	Serial.begin(9600);	// Starting the serial line for viewing via USB
	
	// Bluetooth Setup
	bluetooth.begin(115200);
	Serial.println("Bluetooth Start!");
	bluetooth.println("Bluetooth Start");
	
	// Ultrasonic Setup
	pinMode(trigPin, OUTPUT);	// Set the Ultrasonic trigger pin to o/p
	pinMode(echoPin, INPUT);	// Set the Ultrasonic echo pin to i/p

	Serial.println("Initializing I2C devices...");
	Compass.setpin(keyPin, ledPin);
	delay(200);
	Compass.begin();	// Start the Compass
	delay(200);
	Serial.println("Testing device connections...");
	Serial.println(Compass.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");


	myServo.attach(SERVO);	// Attach the servo

	motor.setSpeed(255); // Speed params 0-255
	motor.run(RELEASE);	// Motor run commands, FORWARDS, BACKWARD & RELEASE (RELEASE STOPS the motor)
}

void loop() 
{
	//debug();
	heading();
	ultrasonic();
	myServo.writeMicroseconds(1500); // Center setting for Rudder
	//fwd();
	
	if (distance > threshold )
	{
		myServo.writeMicroseconds(1500); // Center setting for Rudder
		fwd();
		
		while (angle_fwd > angle_pTol)
		{
			myServo.writeMicroseconds(1200);
		}
		while (angle_fwd < angle_nTol)
		{
			myServo.writeMicroseconds(1700);
		}
	}
	if (distance < threshold)
	{
		turn();
	}
	else
	{
		//debug();
		heading();
		ultrasonic();
		myServo.writeMicroseconds(1500); // Center setting for Rudder
		fwd();
	}
}

void debug()  // Outputs the distance & angle measurements over serial
{
	ultrasonic();
	heading();
	Serial.print("Distance : ");
	Serial.print("\t");
	Serial.print(distance);
	Serial.print(" cm");
	Serial.print("\t\t");
	Serial.print("Heading :");
	Serial.print(angle_number, 1);
	Serial.println(" degree");
	Serial.println("...");
	Serial.println("...");
	//Serial.println("FWD Tolerance");
	//Serial.print("Positive   ");
	//Serial.print(angle_pTol, 1);
	//Serial.print("\t\t");
	//Serial.print("Negative   ");
	//Serial.println(angle_nTol, 1);
	//Serial.println("...");
	//delay(500);
	return;
}

void heading()	// Get the Compass' heading returns heading in degrees
{
	head_X = Compass.getHeadingX();
	head_Y = Compass.getHeadingY();
	head_Z = Compass.getHeadingZ();
	angle_number = Compass.getAngle();

	// Create positive tolerance for Angle FWD
	angle_pTol = (angle_number + Tol);
	if (angle_pTol > 360)
	{
		angle_pTol = angle_pTol - 360;
	}

	// Create negative tolerance for Angle FWD
	angle_nTol = (angle_number - Tol);
	if (angle_nTol < 0)
	{
		angle_nTol = angle_nTol + 360;
	}
	return;
}

void direction()	// Calculate the new heading based on current heading & the tolerances
{
	angle_A = angle_number + 180;
	if (angle_A > 360) // Check to see if the angle is larger than 360
	{
		angle_A = angle_A - 360;
	}

	// Create positive tolerance for Angle A
	angle_A_pTol = (angle_A + Tol);
	if (angle_A_pTol > 360)
	{
		angle_A_pTol = angle_A_pTol - 360;
	}

	// Create negative tolerance for Angle A
	angle_A_nTol = (angle_A - Tol);
	if (angle_A_nTol < 0)
	{
		angle_A_nTol = angle_A_nTol + 360;
	}
	delay(100);
	return;
}

void turn()		// Turn the boat to the new heading
{
	heading();		// Re-calculate the boat's heading
	direction();	// determine the direction (anlgle) the boat needs to turn to
	
	while (angle_number < angle_A_nTol || angle_fwd > angle_A_pTol)		// While the boat is outside the new heading angle turn
	{
		heading();
		turn_speed();
	}
	
}

void fwd()	// Forward movement function
{
	motor.setSpeed(150);	// Motor speed set to 100%
	motor.run(FORWARD);	// Set motor direction
}

void turn_speed()	// Motor speed during a turn function
{
	myServo.writeMicroseconds(1950);	// Move the servo to its full negative value
	motor.setSpeed(255);
	motor.run(FORWARD);
}

long ultrasonic()
{
	//The following trigPin/echoPin cycle is used to determine the dstance of the nearest object by bouncing soundwaves off of it.
	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);

	digitalWrite(trigPin, HIGH);
	delayMicroseconds(5);

	digitalWrite(trigPin, LOW);
	duration = pulseIn(echoPin, HIGH);

	distance = duration / 58.2;		//Calculate the distance (in cm) based on the speed of sound.

	if (distance <= 5 || distance >= 300)
	{
		distance = 300;
	}

	delay(50);		//Delay 50ms before next reading.
}
