// I2Cdev, MPU6050 and BMP180 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include <SFE_BMP180.h>
#include <Wire.h>
#include "PIDCont.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Global.h"
#include "BMP.h"
#include "Init.h"
#include "Smooth.h"

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    initCommunication();
    
    initBMP();
    initMPU();

    PID_init();
    initMotors();
    
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

}

// ===================================================================
// ===                    SUPPORTING FUNCTIONS                     ===
// ===================================================================


void getThrottleValue(){
  
   if (Serial.available()) {
    
    while(Serial.available()){
      delay(2);
      bluetoothChar = Serial.read();
      bluetoothString += bluetoothChar; 
    }

   pIndex = bluetoothString.indexOf('p');
   rIndex = bluetoothString.indexOf('r');
   tIndex = bluetoothString.indexOf('t');

   if (tIndex == 0 && pIndex < 0 && rIndex < 0){
       bluetoothString.remove(0,1);
       tIndex = bluetoothString.indexOf('t');
        if (tIndex > 0)
        {
          bluetoothInt = bluetoothString.substring(0,tIndex).toInt();
        }
        else
        {
          bluetoothInt = bluetoothString.substring(0).toInt();
        }
       if (bluetoothInt <= 100 && bluetoothInt >= 0){
          throttle = map(bluetoothInt,0,100,620,765);
        }
      }        
      
     else if (pIndex == 0 && rIndex > 0 && tIndex < 0){
      setP = -bluetoothString.substring(1,rIndex).toInt();
      bluetoothString.remove(0,rIndex);
      pIndex = bluetoothString.indexOf('p');
        if (pIndex > 0)
        {
          setR = bluetoothString.substring(1,pIndex).toInt();
        }
        else
        {
          setR = bluetoothString.substring(1).toInt();
        }
      }
      
    bluetoothString = "";
  }
   
}

void getPIDValues(){
  setY = 0;
//  setP = 1.48;
//  setR = 0.26;
  
//  PIDyaw_val= (int)PIDyaw.Compute((float)setY-yprdegree[0]);
  PIDyaw_val = 0;
  PIDpitch_val= (int)PIDpitch.Compute((float)setP-digitalSmooth(yprdegree[1],pitchSmoothArray));
  PIDroll_val= (int)PIDroll.Compute((float)setR-digitalSmooth(yprdegree[2],pitchSmoothArray));

}

void adjustMotors(){
  int m1_val=throttle+PIDroll_val+PIDpitch_val+PIDyaw_val;
  int m2_val=throttle+PIDroll_val-PIDpitch_val-PIDyaw_val;
  int m3_val=throttle-PIDroll_val-PIDpitch_val+PIDyaw_val;
  int m4_val=throttle-PIDroll_val+PIDpitch_val-PIDyaw_val;

#ifdef debug
//  Serial.print("M1: ");
//  Serial.print(m1_val);
//  Serial.print(", M2: ");
//  Serial.print(m2_val);
//  Serial.print(", M3: ");
//  Serial.print(m3_val);
//  Serial.print(", M4: ");
//  Serial.println(m4_val);
#endif

  if (throttle == MOTOR_ZERO_LEVEL) m1_val = MOTOR_ZERO_LEVEL;
  else if (m1_val < MOTOR_RUN_LEVEL) m1_val = MOTOR_RUN_LEVEL;
  else if(m1_val >= MAX_SIGNAL) m1_val = MAX_SIGNAL;

  if (throttle == MOTOR_ZERO_LEVEL) m2_val = MOTOR_ZERO_LEVEL;
  else if (m2_val < MOTOR_RUN_LEVEL) m2_val = MOTOR_RUN_LEVEL;
  else if(m2_val >= MAX_SIGNAL) m2_val = MAX_SIGNAL;

  if (throttle == MOTOR_ZERO_LEVEL) m3_val = MOTOR_ZERO_LEVEL;
  else if (m3_val < MOTOR_RUN_LEVEL) m3_val = MOTOR_RUN_LEVEL;
  else if(m3_val >= MAX_SIGNAL) m3_val = MAX_SIGNAL;

  if (throttle == MOTOR_ZERO_LEVEL) m4_val = MOTOR_ZERO_LEVEL;
  else if (m4_val < MOTOR_RUN_LEVEL) m4_val = MOTOR_RUN_LEVEL;
  else if(m4_val >= MAX_SIGNAL) m4_val = MAX_SIGNAL;

#ifdef debug
//  Serial.print("Current Yaw: ");
//  Serial.print(yprdegree[0]);
//  Serial.print(", Current Pitch: ");
//  Serial.print(yprdegree[1]);
//  Serial.print(", Current Roll: ");
//  Serial.println(yprdegree[2]);
//
//  Serial.print("Yaw Error: ");
//  Serial.print((float)setY-yprdegree[0]);
//  Serial.print(", Pitch Error: ");
//  Serial.print((float)setP-yprdegree[1]);
//  Serial.print(", Roll Erro: ");
//  Serial.println((float)setR-yprdegree[2]);
//
//  Serial.print("Yaw PID: ");
//  Serial.print(PIDyaw_val);
//  Serial.print(", Pitch PID: ");
//  Serial.print(PIDpitch_val);
//  Serial.print(", Roll PID: ");
//  Serial.println(PIDroll_val);
//
//  Serial.print("M1: ");
//  Serial.print(m1_val);
//  Serial.print(", M2: ");
//  Serial.print(m2_val);
//  Serial.print(", M3: ");
//  Serial.print(m3_val);
//  Serial.print(", M4: ");
//  Serial.println(m4_val);
#endif

  analogWrite(MOTOR1,m1_val);
  analogWrite(MOTOR2,m2_val);
  analogWrite(MOTOR3,m3_val);
  analogWrite(MOTOR4,m4_val);
}

void updateSensors() {
    double a,P;
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
      
    // wait for MPU interrupt or extra packet(s) available
    do {
      P = getPressure();
      a = pressure.altitude(P,baseline);
      #ifdef debug
        Serial.print("relative altitude: ");
        if (a >= 0.0) Serial.print(" "); // add a space for positive numbers
        if (a <= 0.0) a = 0;
        Serial.print(a,1);
        Serial.print(" meters, ");
        if (a >= 0.0) Serial.print(" "); // add a space for positive numbers
        Serial.print(a*3.28084,0);
        Serial.println(" feet");
      #endif
    }
    while (!mpuInterrupt && fifoCount < packetSize);

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.resetFIFO();
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yprdegree[0] = (ypr[0] * 180/M_PI);
            yprdegree[1] = (ypr[1] * 180/M_PI);
            yprdegree[2] = (ypr[2] * 180/M_PI);
            #ifdef debug
              Serial.print("ypr\t");
              Serial.print(ypr[0] * 180/M_PI);
              Serial.print("\t");
              Serial.print(ypr[1] * 180/M_PI);
              Serial.print("\t");
              Serial.println(ypr[2] * 180/M_PI);
            #endif
            
        
    }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================


void loop(){
  updateSensors();
  getThrottleValue();
  getPIDValues();
  adjustMotors();
}
