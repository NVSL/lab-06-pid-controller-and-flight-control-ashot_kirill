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

int mot_pin = 3;

unsigned long time;

double Setpoint, Input, Output;

double consKp = 0.2, consKi = 2, consKd = 0;

double aggKp = 0.3, aggKi = -0.5, aggKd = 0.1;

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);

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
  
  Input = 0;
  Setpoint = 0;
  Output = 0;
  // Setup the sensor gain and integration time.
  configureLSM9DS0();
  
  pinMode(mot_pin,OUTPUT);
  digitalWrite(mot_pin,LOW);
  
  time = millis();
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  //myPID.SetOutputLimits(-255, 255);
}

void loop(void) 
{
  sensors_vec_t   orientation;

  if(rfAvailable())
  {
      Setpoint = -1 * rfRead();
      //Serial.println(var);
  }
  
  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation) && 1)//millis() - time > 200)
  {
    time = millis();
    /* 'orientation' should have valid .roll and .pitch fields */
    Input = -1 * orientation.roll;
    
    double gap = abs(Setpoint-Input); //distance away from setpoint
    if (gap < 10)
      myPID.SetTunings(consKp, consKi, consKd);
    else
      myPID.SetTunings(aggKp, aggKi, aggKd);
    
    myPID.Compute();
    
    
    Serial.print(F("Orientation: "));
    Serial.print(Input);
    Serial.print(" PID Output: ");
    Serial.print(Output);
    Serial.println(); 
  }
  
    analogWrite(mot_pin, Output);
}
