#include <SPI.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <Adafruit_Simple_AHRS.h>
#include "RadioFunctions.h"

// Create LSM9DS0 board instance.
Adafruit_LSM9DS0     lsm(1000);  // Use I2C, ID #1000

// Create simple AHRS algorithm using the LSM9DS0 instance's accelerometer and magnetometer.
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag());

// Function to configure the sensors on the LSM9DS0 board.
// You don't need to change anything here, but have the option to select different
// range and gain values.
void configureLSM9DS0(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

int m1 = 5;
int m2 = 4;
int m3 = 3;
int m4 = 8;

unsigned long time;

double roll_setpoint, roll_input, roll_output;
double pitch_setpoint, pitch_input, pitch_output;
double yaw_setpoint, yaw_input, yaw_output;

double th;

//Roll Coef
double consKp = 0.2, consKi = 2, consKd = 0;
double aggKp = 0.3, aggKi = -0.5, aggKd = 0.1;

PID roll_PID(&roll_input, &roll_output, &roll_setpoint, consKp, consKi, consKd, DIRECT);
PID pitch_PID(&pitch_input, &pitch_output, &pitch_setpoint, consKp, consKi, consKd, DIRECT);
PID yaw_PID(&yaw_input, &yaw_output, &yaw_setpoint, consKp, consKi, consKd, DIRECT);

void setup(void) 
{
  Serial.begin(115200);
  Serial.println(F("Adafruit LSM9DS0 9 DOF Board AHRS Example")); Serial.println("");
  rfBegin(11);
  // Initialise the LSM9DS0 board.
  if(!lsm.begin())
  {
    // There was a problem detecting the LSM9DS0 ... check your connections
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  
  roll_input = 0; roll_setpoint = 0; roll_output = 0;
  yaw_input = 0; yaw_setpoint = 0; yaw_output = 0;
  pitch_input = 0; pitch_setpoint = 0; pitch_output = 0;
  
  th = 0;
  // Setup the sensor gain and integration time.
  configureLSM9DS0();
 
  time = millis();
  //turn the PID on
  roll_PID.SetMode(AUTOMATIC);
  //myPID.SetOutputLimits(-255, 255);
}

void loop(void) 
{
  sensors_vec_t   orientation;

  if(rfAvailable())
  {
      roll_setpoint = -1 * rfRead();
      //Serial.println(var);
  }
  
  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation) && 1)//millis() - time > 200)
  {
    time = millis();
    /* 'orientation' should have valid .roll and .pitch fields */
    roll_input = orientation.roll;
    yaw_input = orientation.heading;
    pitch_input = orientation.pitch;
    
    //roll
    double gap = abs(roll_setpoint-roll_input); //distance away from setpoint
    if (gap < 10)
      roll_PID.SetTunings(consKp, consKi, consKd);
    else
      roll_PID.SetTunings(aggKp, aggKi, aggKd);
    roll_PID.Compute();
    
    //pitch
    gap = abs(pitch_setpoint-pitch_input); //distance away from setpoint
    if (gap < 10)
      pitch_PID.SetTunings(consKp, consKi, consKd);
    else
      pitch_PID.SetTunings(aggKp, aggKi, aggKd);
    pitch_PID.Compute();
    
    
    Serial.print("Roll: ");
    Serial.print(roll_input);
    Serial.print(" Pitch: ");
    Serial.print(pitch_input);
    Serial.print(" Yaw: ");
    Serial.print(yaw_input);
    Serial.println();
  }
  
//  analogWrite(m1, th + roll_output/2 - pitch_output/2);
//  analogWrite(m2, th - roll_output/2 - pitch_output/2);
//  analogWrite(m3, th - roll_output/2 + pitch_output/2);
//  analogWrite(m4, th + roll_output/2 + pitch_output/2);
}
