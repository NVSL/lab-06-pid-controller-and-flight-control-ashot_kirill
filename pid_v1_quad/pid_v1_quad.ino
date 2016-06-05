

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

union {
  struct stick{
    uint16_t ROLL;
    uint16_t PITCH;
    uint16_t YAW;
    uint16_t THROTTLE;
    uint16_t AUX1;
    uint16_t AUX2;
    uint16_t AUX3;
    uint16_t AUX4;
  } 
  m;
  uint16_t rc_channels[8];
  char buff[sizeof(struct stick)];
} 
stick_struct;

char input[sizeof(stick_struct)];

uint16_t ROLL = 1500;
uint16_t PITCH = 1500;
uint16_t YAW = 1500;
uint16_t THROTTLE = 1000;

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
  rfBegin(15);
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
  Serial.println("here! 1");
  th = 0;
  // Setup the sensor gain and integration time.
  configureLSM9DS0();
  Serial.println("here! 2");
 
  time = millis();
  //turn the PID on
  roll_PID.SetMode(AUTOMATIC);
  pitch_PID.SetMode(AUTOMATIC);
  roll_PID.SetOutputLimits(-255, 255);
  pitch_PID.SetOutputLimits(-255, 255);
}

void loop(void) 
{
  sensors_vec_t   orientation;

  if(rfAvailable())
  {
//      roll_setpoint = -1 * rfRead();
      int i = 0;
      char c = rfRead();
      while(c != -1 && c != '\0'){
          Serial.print(c);
          input[i++] = c;
          c = rfRead();
      }
      //input[i] = c;      
      memcpy(&ROLL, &input[0], sizeof(uint16_t));
      memcpy(&PITCH, &input[2], sizeof(uint16_t));
      memcpy(&YAW, &input[4], sizeof(uint16_t));
      memcpy(&THROTTLE, &input[6], sizeof(uint16_t));
//      Serial.print("#RC Roll: ");
//      Serial.print(int(ROLL));
//      Serial.print(" Pitch: ");
//      Serial.print(int(PITCH));
//      Serial.print(" Yaw: ");
//      Serial.print(int(YAW));
//      Serial.print(" Throttle: ");
//      Serial.println(int(THROTTLE));
      //Serial.println(var);
  }
  
  th = map(THROTTLE, 1000, 2000, 0, 200);
  // Use the simple AHRS function to get the current orientation.
  if (ahrs.getOrientation(&orientation) && 1)//millis() - time > 200)
  {
    time = millis();
    /* 'orientation' should have valid .roll and .pitch fields */
    pitch_input = (int)orientation.roll;
    yaw_input = (int)orientation.heading;
    roll_input = (int)orientation.pitch;
    
    //roll
    double gap = abs(roll_setpoint-roll_input); //distance away from setpoint
    if (gap < 10)
      roll_PID.SetTunings(consKp, consKi, consKd);
    else
      roll_PID.SetTunings(aggKp, aggKi, aggKd);
    
    //pitch
    gap = abs(pitch_setpoint-pitch_input); //distance away from setpoint
    if (gap < 10)
      pitch_PID.SetTunings(consKp, consKi, consKd);
    else
      pitch_PID.SetTunings(aggKp, aggKi, aggKd);
    if(th > 5){
      roll_PID.Compute();
      pitch_PID.Compute();
    }
  }
//  Serial.print("*IMU Roll: ");
//  Serial.print(roll_input);
//  Serial.print(" Pitch: ");
//  Serial.print(pitch_input);
//  Serial.print(" Yaw: ");
//  Serial.print(yaw_input);
//  Serial.println();
//
//  Serial.print("roll_output: ");
//  Serial.print(roll_output);
//  Serial.print(" pitch_output: ");
//  Serial.println(pitch_output);
//
//  Serial.print("m1: ");
//  Serial.print((th - roll_output + pitch_output) > 0?(th - roll_output + pitch_output):0);
//  Serial.print(" m2: ");
//  Serial.print((th - roll_output) > 0?(th - roll_output):0);
//  Serial.print(" m3: ");
//  Serial.print(th);
//  Serial.print(" m4: ");
//  Serial.println((th + pitch_output) > 0?(th + pitch_output):0);
//  Serial.println("_____________________________________________");
//  
//  
  analogWrite(m1, (th - roll_output + pitch_output) > 0?(th - roll_output + pitch_output):0);
  analogWrite(m2, (th - roll_output) > 0?(th - roll_output):0);
  analogWrite(m3, th);
  analogWrite(m4, (th + pitch_output) > 0?(th + pitch_output):0);
  }


