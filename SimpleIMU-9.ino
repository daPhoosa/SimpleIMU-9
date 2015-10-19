/*  
 *  Very Simple Complimentary Filter for the InvenSense MPU-6050 and HMC5883L
 *  Phillip Schmidt
 *  v1.0, June 2015
 */

#include <Wire.h>
#include <MPU6050.h>
#include <HMC5883L.h>
#include <Math3D.h>
#include <PollTimer.h>

#define _DEGREES(x) (57.29578f * x)
#define _RADIANS(x) (0.017453293f * x)


#define GYRO_AXIS_MAPPING 	MPU.gX, MPU.gY, MPU.gZ	// (adjust sensor axis alignment/direction here)
#define ACCEL_AXIS_MAPPING	MPU.aX, MPU.aY, MPU.aZ
#define MAG_AXIS_MAPPING	  MAG.X,  MAG.Y,  MAG.Z


MPU6050 MPU(400, 0, 3, 3); // update rate, filtering, gyro range, accel range
HMC5883L MAG; // magnetometer object 

PollTimer RateLoopTimer(400UL);
PollTimer AttitudeLoopTimer(100UL);
PollTimer NavLoopTimer(25UL);
PollTimer MaintenanceLoop(1UL);

Quat AttitudeEstimateQuat;

Vec3 correction_Body, correction_World;
Vec3 Accel_Body, Accel_World;
Vec3 RotationRateVec, driftCorrection;

// magnetic declination in decimal degrees ( '+' = EAST, '-' = WEST )
const float declination = _RADIANS(0.36f);	// this must be adjusted to your flying location
	
const Vec3 VEC_NORTH = Vector(cos(declination), sin(declination), 0.0f);  // vertical vector in the World frame, corrected for magnetic declination
const Vec3 VEC_VERTICAL = Vector(0.0f, 0.0f, 1.0f);  // vertical vector in the World frame
const Vec3 VEC_ZEROES = Vector(0.0f, 0.0f, 0.0f);

unsigned long testTimer;

void setup() // initialization and start-up operations
{

	Serial.begin(115200);  // start serial for output
	Serial.println("*** Run On Start ***");

	Wire.begin();        // join i2c bus
	Wire.setClock(400000UL);// set speed to 400k


	MPU.initialize();
	Serial.println(MPU.samplePeriod);
	MPU.accelZero();  // generate and store accel bias offsets
	MPU.gyroZero();	  // generate and store gyro bias offsets

  MAG.begin(-29, 216, 4); // (X, Y, Z biases)

	RateLoopTimer.start();
	AttitudeLoopTimer.start();
	NavLoopTimer.start();
	MaintenanceLoop.start();
	
}

void loop() // Start Main Loop
{

	/*
	 *		Note: Control loops are nested as else-if to allow faster loops
	 *			to gain priority after any single lower rate loop executes.
	 *			This prevents multiple low priority loops from excessively 
	 *			delaying the execution of a high priority loop.
	 *			(programmer attention is required to prevent saturation due
	 *			 to excessively long operations in higher frequency loops)
	 */


	if(RateLoopTimer.check())  // Run this loop at 400Hz - Accel/Gyro reading, Attitude integration & Aircraft rate control
	{
		//testTimer = micros();
    
		// get Accel/Gyro data from the sensor
		MPU.retrieve();

		// move gyro data to vector structure, sum with correction data 
		RotationRateVec = Sum(RotationRateVec, Vector(GYRO_AXIS_MAPPING));	
    	
		// create incremental rotation quat
		Quat incrementalRotation = Quaternion(RotationRateVec, MPU.samplePeriod);  
    
		// quaternion integration (rotation composting through multiplication)		
		AttitudeEstimateQuat = Mul(incrementalRotation, AttitudeEstimateQuat);  
		
		// set rotation rate vector to zero after integration
		RotationRateVec = VEC_ZEROES;
		
		
		// move accel data to vector structure 
		Accel_Body = Vector(ACCEL_AXIS_MAPPING);	
		
		// rotate accel from body frame to world frame, sum with previous data (summing multiple samples to act as LPF)
		Accel_World = Sum(Accel_World, Rotate(AttitudeEstimateQuat, Accel_Body)); 

		//	Note: corrections will be computed in the slower loop
		
		// FUTURE: Aircraft Rotation Rate control will go here
    
		//testTimer = micros() - testTimer;
	}
	else if(AttitudeLoopTimer.check())	// Run this loop at 100Hz - Magnetometer reading, Attitude estimate correction & Aircraft attitude control
	{
		testTimer = micros();
	
		// Accel cross product to determine indicated pitch/roll error
		correction_World = CrossProd(Accel_World, VEC_VERTICAL); 
		
		// FUTURE: Accel integration for vel and pos estimation (with GPS and Baro)
		
		// reset	accel reading accumulator
		Accel_World = VEC_ZEROES;
		
		
		// Retrieve Magnetometer data
		MAG.getData();
		
		// move Mag data to vector structure
		Vec3 Mag_Body = Vector(MAG_AXIS_MAPPING);
		
		// Rotate Mag data to world frame
		Vec3 Mag_World = Rotate(AttitudeEstimateQuat, Mag_Body);
		
		// Remove inclination in Mag data
		Mag_World.z = 0.0f;
		
		// Normalize Mag data
		Mag_World = Normalize(Mag_World);
		
		// Mag cross VEC_NORTH to determine indicated yaw error, sum with Accel correction data
		correction_World = Sum(correction_World, CrossProd(Mag_World, VEC_NORTH));
		
		// rotate correction vector to body frame
		Vec3 correction_Body = Rotate(correction_World, AttitudeEstimateQuat); 

		// add correction vector to rotation rate data
		RotationRateVec = Sum(RotationRateVec, correction_Body);
		
		// long term drift correction in body frame
		//driftCorrection = Sum(driftCorrection, RotationRateVec); // accumulate error
		//driftCorrection.x = constrain(driftCorrection.x, -100.0f, 100.0f);
		//driftCorrection.y = constrain(driftCorrection.y, -100.0f, 100.0f);
		//driftCorrection.z = constrain(driftCorrection.z, -200.0f, 200.0f);
		//RotationRateVec = Sum(RotationRateVec, Mul(driftCorrection, 0.001f)); // apply a small amount of the error to the correction
		
		
				
		// FUTURE: Aircraft Attitude control will go here

		testTimer = micros() - testTimer;
	}
	else if(NavLoopTimer.check())
	{
		// FUTURE: GPS parsing here, navigation
	}
	else if(MaintenanceLoop.check())	// only display data occasionally since it is very time consuming
	{
		
		Normalize(AttitudeEstimateQuat);	// normalize orientation quaternion
		
		Vec3 YPR = YawPitchRoll(AttitudeEstimateQuat);
		Serial.print("  Yaw:");   Serial.print(_DEGREES(-YPR.x), 2);
		Serial.print("  Pitch:"); Serial.print(_DEGREES(-YPR.y), 2);
		Serial.print("  Roll:");Serial.println(_DEGREES(-YPR.z), 2);

		//display(AttitudeEstimateQuat);
		//display(RotationRateVec);
		//display(AccelVec);
    display(driftCorrection);

    Serial.println(testTimer, DEC);
	}

} // Main Loop End
