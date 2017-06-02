// I2Cdev, MPU6050 and BMP180 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project

//gyro.x = roll, gyro.y = pitch, gyro.z = yaw. 
#include "I2Cdev.h"
#include <Wire.h>
#include <Servo.h>
#include "PIDCont.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Global.h"
#include "Init.h"
#include "Smooth.h"

//#define BMP 0
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    initCommunication();
    
    //initBMP();
    initMPU();

    PID_init();
    initMotors();
    
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    timeOfLastSignal = millis();
}

// ===================================================================
// ===                    SUPPORTING FUNCTIONS                     ===
// ===================================================================

bool rateAndControl = false;
bool changePID = false;
bool debug = false;

String temp = "";

void updateIndexes(){
  int commaIndex = -1;

  endIndex = bluetoothString.length();
  commaIndex = bluetoothString.indexOf(',');
  temp = bluetoothString.substring(0,commaIndex);  
  bluetoothString = bluetoothString.substring(commaIndex + 1,endIndex);
}

void processString(){
  //Failsafe,Debug,PID,Throttle,rateAndControl,PhoneYaw,PhonePitch,PhoneRoll,RP_P,RP_I,RP_D,Y_P,Y_I,Y_D|
  //,0,0,0,0,0,0,0.25,0.0,0.0,0.0,-0.01,0.0|
  int bluetoothInt;
  
  updateIndexes();
  if (temp.charAt(0) == 'C')
  {
    if (bluetoothString.length() != temp.substring(1,temp.length()).toInt())
    {
      bluetoothString = "";
      return;
    }
  }
  else
  {
    return;
  }

  if (bluetoothString.substring(bluetoothString.length()-1,bluetoothString.length()) == "|")
  {
      timeOfLastSignal = millis();
      
      //Failsafe
      updateIndexes();
      //Flag to turn off failsafe
      if (temp == "1") failSafe = false;
      //failSafe = false;
      //    else failSafe = false;
  
      //Debug
      updateIndexes();
      if (temp == "1") debug = true;
      else debug = false;
  
      //PID
      updateIndexes();
      if (temp == "1") changePID = true;
      else changePID = false;
  
      //Throttle
      updateIndexes();
      bluetoothInt = temp.toInt();
      
      if (failSafe)
      {
        throttle = map(0,0,100,MOTOR_ZERO_LEVEL,MAX_SIGNAL);
      }
      else if (bluetoothInt <= 100 && bluetoothInt >= 0)
      {
        throttle = map(bluetoothInt,0,100,MOTOR_ZERO_LEVEL,MAX_SIGNAL);
      }
      
      //rateAndControl
      updateIndexes();
      if (temp == "1") rateAndControl = true;
      else rateAndControl = false;
  
      updateIndexes();
      receivedYaw = temp.toInt();
      updateIndexes();
      receivedPitch = temp.toInt();
      updateIndexes();
      receivedRoll = temp.toInt();
  
      setY = receivedYaw;
      setR = receivedRoll;
      setP = receivedPitch;
  
      char floatbuf[32]; // make this at least big enough for the whole string
      double toChange = 0.0;
  
      //Roll and Pitch PID
      updateIndexes();
      if(changePID)
      {
        temp.toCharArray(floatbuf, sizeof(floatbuf));
        toChange = atof(floatbuf);
        ROLL_PID_KP = toChange;
        PITCH_PID_KP = toChange;
      }
      
      updateIndexes();
      if(changePID)
      {
        temp.toCharArray(floatbuf, sizeof(floatbuf));
        toChange = atof(floatbuf);
        ROLL_PID_KI = toChange;
        PITCH_PID_KI = toChange;
      }
      
      updateIndexes();
      if(changePID)
      {
        temp.toCharArray(floatbuf, sizeof(floatbuf));
        toChange = atof(floatbuf);
        ROLL_PID_KD = toChange;
        PITCH_PID_KD = toChange;
      }
      
      //YAW PID
      updateIndexes();
      if(changePID)
      {
        temp.toCharArray(floatbuf, sizeof(floatbuf));
        toChange = atof(floatbuf);
        YAW_PID_KP = toChange;
      }
      
      updateIndexes();
      if(changePID)
      {
        temp.toCharArray(floatbuf, sizeof(floatbuf));
        toChange = atof(floatbuf);
        YAW_PID_KI = toChange;
      }
      
      updateIndexes();
      if(changePID)
      {
        temp.toCharArray(floatbuf, sizeof(floatbuf));
        toChange = atof(floatbuf);
        YAW_PID_KD = toChange;
      }
  
      //SET PID
      if (changePID)
      {
        PIDroll.resetI();
        PIDpitch.resetI();
        PIDyaw.resetI();
        PIDangleX.resetI();
        PIDangleY.resetI();
        PIDroll.ChangeParameters(ROLL_PID_KP,ROLL_PID_KI,ROLL_PID_KD,ROLL_PID_MIN,ROLL_PID_MAX);
        PIDpitch.ChangeParameters(PITCH_PID_KP,PITCH_PID_KI,PITCH_PID_KD,PITCH_PID_MIN,PITCH_PID_MAX);
        PIDyaw.ChangeParameters(YAW_PID_KP,YAW_PID_KI,YAW_PID_KD,YAW_PID_MIN,YAW_PID_MAX);

        Serial.println(F("Roll"));
        Serial.println("P: " + String(ROLL_PID_KP) + " I: " + String(ROLL_PID_KI) + " D: " + String(ROLL_PID_KD) + " m: " + String(ROLL_PID_MIN) + " M: " + String(ROLL_PID_MAX));
        Serial.println(F("Pitch"));
        Serial.println("P: " + String(PITCH_PID_KP) + " I: " + String(PITCH_PID_KI) + " D: " + String(PITCH_PID_KD) + " m: " + String(PITCH_PID_MIN) + " M: " + String(PITCH_PID_MAX));
        Serial.println(F("Yaw"));
        Serial.println("P: " + String(YAW_PID_KP) + " I: " + String(YAW_PID_KI) + " D: " + String(YAW_PID_KD) + " m: " + String(YAW_PID_MIN) + " M: " + String(YAW_PID_MAX));
      }
  
      //reset
      endIndex = 0;
      temp = "";      
  }
  bluetoothString = "";
}

void getBluetoothData(){
  if (Serial.available())
  {
    int upper = min(bluetoothDataLength,Serial.available());
    
    for (int byteCounter = 0 ; byteCounter < upper ; byteCounter ++)
    {
      bluetoothChar = Serial.read();
      
      bluetoothString += bluetoothChar; 
      
      if(bluetoothChar == '|')
      {
        processString();
        
        while(Serial.available())
        {
          Serial.read();
        }
        
        break;
      }//bluetoothChar
    }//For Loop
  }//Serial.available
}

void getPIDValues(){
  
  if (rateAndControl == true)
  { 
    setP=(int)PIDangleY.Compute(receivedPitch-angles[0]); 
    setR=(int)PIDangleX.Compute(receivedRoll+angles[1]);  
  } 
  
  PIDroll_val= (int)PIDroll.Compute((double)(setR-gy_aver)); 
  PIDpitch_val= (int)PIDpitch.Compute((double)(setP-gx_aver)); 
  PIDyaw_val= (int)PIDyaw.Compute((double)(wrap_180(setY-gz_aver)));
}

void adjustMotors(){
  
  m1_val =throttle-PIDroll_val+PIDpitch_val+PIDyaw_val;
  m2_val=throttle-PIDroll_val-PIDpitch_val-PIDyaw_val;
  m3_val=throttle+PIDroll_val-PIDpitch_val+PIDyaw_val;
  m4_val=throttle+PIDroll_val+PIDpitch_val-PIDyaw_val;

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

  printStuff();
  MOTOR1.writeMicroseconds(m1_val);
  MOTOR2.writeMicroseconds(m2_val);
  MOTOR3.writeMicroseconds(m3_val);
  MOTOR4.writeMicroseconds(m4_val);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void printStuff(){
  if (debug && millis() - printTimer > PRINT_TIME)
  {

    Serial.print("M1:" + String(m1_val) + "M2:" + String(m2_val) + "M3:" + String(m3_val) + "M4:" + String(m4_val));
    Serial.print(" recY: " + String(receivedYaw) + " recP: " + String(receivedPitch) + " recR: " + String(receivedRoll));

    Serial.print("P:" + String(angles[0]) + " R:" + String(angles[1]));

    Serial.print("smGY:" + String(gz_aver) + "smGP:" + String(gx_aver) + "smGR:" + String(gy_aver));

    //Needed so that the phone knows to reset throttle when failsafe is triggered by drone
    //Serial.print(" F:" + String(failSafe));
    Serial.println("");
    printTimer = millis();
    
  }
}
void loop(){
  
  //failsafe
  if (millis() - timeOfLastSignal > FAILSAFE_THRESHOLD && !failSafe)
  {
    failSafe = true;
    Serial.println(F("No Signal"));
  }
  
  //1-6 millis
  if (millis()-slowLoopTimer >= SLOW_SAMPLE_TIME)
  {
    getBluetoothData();
   
    if (failSafe) 
    Serial.println(F("FAIL!"));

    updateAcc();
    
    slowLoopTimer = millis();
  }

  if (micros()-fastLoopTimer >= FAST_SAMPLE_TIME)
  {
    //400 Micro, Max 512
    getPIDValues();
    
    //36 Micro, Max 64
    adjustMotors();
    
    updateGyroData();
    updateSensorVal();
    fastLoopTimer = micros();
  }
}
