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

}

// ===================================================================
// ===                    SUPPORTING FUNCTIONS                     ===
// ===================================================================

bool receivedAll = false;
bool debug = false;
bool control = false;
bool changePID = false;

int commaIndex = -1;
String temp = "";

void updateIndexes(){
  endIndex = bluetoothString.length();
  commaIndex = bluetoothString.indexOf(',');
  temp = bluetoothString.substring(0,commaIndex);  
  bluetoothString = bluetoothString.substring(commaIndex + 1,endIndex);
}

void processString(){
//Debug,Calibrate,PID,Throttle,Control,PhoneYaw,PhonePitch,PhoneRoll,RP_P,RP_I,RP_D,Y_P,Y_I,Y_D|
//,0,0,0,0,0,0,0.25,0.0,0.0,0.0,-0.01,0.0|
  int bluetoothInt;
//  Serial.println(bluetoothString);
//  if (debug)Serial.println("Length: " + String(bluetoothString.length()) + " " + bluetoothString);
  //Max length of a string should be 59 but i made it 61 just in case
//  Serial.println ( bluetoothString + " - " + );
  if (bluetoothString.length() < 61  && 
  bluetoothString.substring(bluetoothString.length()-1,bluetoothString.length()) == "|"
  ){
    //Debug
    updateIndexes();
    if (temp == "1") debug = true;
    else debug = false;

    //Calibrate
    updateIndexes();
    if (temp == "1") {
        setY = yprdegree[0];
        setP = yprdegree[1];
        setR = yprdegree[2];
        prevY = setY;
        prevP = setP;
        prevR = setR;


//YPR degree is not updating here for some reason :(
//  Serial.print("Y: ");
//  Serial.print(yprdegree[0]);
//  Serial.print(", P: ");
//  Serial.print(yprdegree[1]);
//  Serial.print(", R: ");
//  Serial.println(yprdegree[2]);
        }
        
    else {      if (control == false){
        //TODO SAVE CALIBRATE VALUE AND SET IT HERE
        setY = prevY;
        setP = prevP;
        setR = prevR;
        }
    }


    //PID
    updateIndexes();
    if (temp == "1") changePID = true;
    else changePID = false;

    //Throttle
    updateIndexes();
    bluetoothInt = temp.toInt();
    if (bluetoothInt <= 100 && bluetoothInt >= 0){
      throttle = map(bluetoothInt,0,100,620,765);
    }

    //Control
    updateIndexes();
    if (temp == "1") control = true;
    else control = false;

    //Phone yaw pitch roll
    if (control){
      updateIndexes();
      setY = temp.toInt();
      updateIndexes();
      setP = temp.toInt();
      updateIndexes();
      setR = temp.toInt();
    }
    else{
      updateIndexes();
      updateIndexes();
      updateIndexes();
    }

    char floatbuf[32]; // make this at least big enough for the whole string
    double toChange = 0.0;

    //Roll and Pitch PID
    updateIndexes();
    temp.toCharArray(floatbuf, sizeof(floatbuf));
    toChange = atof(floatbuf);
    ROLL_PID_KP = toChange;
    PITCH_PID_KP = toChange;
    updateIndexes();
    temp.toCharArray(floatbuf, sizeof(floatbuf));
    toChange = atof(floatbuf);
    ROLL_PID_KI = toChange;
    PITCH_PID_KI = toChange;
    updateIndexes();
    temp.toCharArray(floatbuf, sizeof(floatbuf));
    toChange = atof(floatbuf);
    ROLL_PID_KD = toChange;
    PITCH_PID_KD = toChange;

    //YAW PID
    updateIndexes();
    temp.toCharArray(floatbuf, sizeof(floatbuf));
    toChange = atof(floatbuf);
    YAW_PID_KP = toChange;
    updateIndexes();
    temp.toCharArray(floatbuf, sizeof(floatbuf));
    toChange = atof(floatbuf);
    YAW_PID_KI = toChange;
    updateIndexes();
    temp.toCharArray(floatbuf, sizeof(floatbuf));
    toChange = atof(floatbuf);
    YAW_PID_KD = toChange;

    //SET PID
    //                  //                          Kp,        Ki,         Kd           Lval         Hval
    if (changePID){
      PIDroll.resetI();
      PIDpitch.resetI();
      PIDyaw.resetI();
    PIDroll.ChangeParameters(ROLL_PID_KP,ROLL_PID_KI,ROLL_PID_KD,ROLL_PID_MIN,ROLL_PID_MAX);
    PIDpitch.ChangeParameters(PITCH_PID_KP,PITCH_PID_KI,PITCH_PID_KD,PITCH_PID_MIN,PITCH_PID_MAX);
    PIDyaw.ChangeParameters(YAW_PID_KP,YAW_PID_KI,YAW_PID_KD,YAW_PID_MIN,YAW_PID_MAX);
//    }
//
//    if (debug){
        Serial.println("Roll");
        String output = "P: " + String(ROLL_PID_KP) + " I: " + String(ROLL_PID_KI) + " D: " + String(ROLL_PID_KD) + " m: " + String(ROLL_PID_MIN) + " M: " + String(ROLL_PID_MAX);
        Serial.println(output);
        Serial.println("Pitch");
        output = "P: " + String(PITCH_PID_KP) + " I: " + String(PITCH_PID_KI) + " D: " + String(PITCH_PID_KD) + " m: " + String(PITCH_PID_MIN) + " M: " + String(PITCH_PID_MAX);
        Serial.println(output);
        Serial.println("Yaw");
        output = "P: " + String(YAW_PID_KP) + " I: " + String(YAW_PID_KI) + " D: " + String(YAW_PID_KD) + " m: " + String(YAW_PID_MIN) + " M: " + String(YAW_PID_MAX);
        Serial.println(output);
    }

    //reset
    endIndex = 0;
    commaIndex = 0;
    temp = "";
    bluetoothString = "";
  }
  String num;
  
//   pIndex = bluetoothString.indexOf('P');
//   iIndex = bluetoothString.indexOf('I');
//   dIndex = bluetoothString.indexOf('D');
//   endIndex = bluetoothString.indexOf('|');
//   tIndex = bluetoothString.indexOf('t');
//
//   if (tIndex == 0 && pIndex < 0 && iIndex < 0 && dIndex < 0){
//       bluetoothString.remove(0,1);
//       tIndex = bluetoothString.indexOf('t');
//        if (tIndex > 0)
//        {
//          bluetoothInt = bluetoothString.substring(0,tIndex).toInt();
//        }
//        else
//        {
//          bluetoothInt = bluetoothString.substring(0).toInt();
//        }
//       if (bluetoothInt <= 100 && bluetoothInt >= 0){
//          throttle = map(bluetoothInt,0,100,620,765);
//        }
//      }        
//      
//     else if (endIndex > 0){
//        num = bluetoothString.substring(2,endIndex);
////        Serial.println(num);
//        num.toCharArray(floatbuf, sizeof(floatbuf));
//        double toChange = atof(floatbuf);
////        Serial.println(toChange);
//        if(pIndex == 0){
//          ROLL_PID_KP = toChange;
//          PITCH_PID_KP = toChange;
//        }
//        else if (iIndex == 0){
//          ROLL_PID_KI = toChange;
//          PITCH_PID_KI = toChange;
//        }
//        else if (dIndex == 0){
//          ROLL_PID_KD = toChange;
//          PITCH_PID_KD = toChange;
//        }
//                  //                          Kp,        Ki,         Kd           Lval         Hval
//        PIDroll.ChangeParameters(ROLL_PID_KP,ROLL_PID_KI,ROLL_PID_KD,ROLL_PID_MIN,ROLL_PID_MAX);
//        PIDpitch.ChangeParameters(PITCH_PID_KP,PITCH_PID_KI,PITCH_PID_KD,PITCH_PID_MIN,PITCH_PID_MAX);
//        PIDyaw.ChangeParameters(YAW_PID_KP,YAW_PID_KI,YAW_PID_KD,YAW_PID_MIN,YAW_PID_MAX);
//        Serial.println("Roll");
//        String output = "P: " + String(ROLL_PID_KP) + " I: " + String(ROLL_PID_KI) + " D: " + String(ROLL_PID_KD) + " m: " + String(ROLL_PID_MIN) + " M: " + String(ROLL_PID_MAX);
//        Serial.println(output);
//        Serial.println("Pitch");
//        output = "P: " + String(PITCH_PID_KP) + " I: " + String(PITCH_PID_KI) + " D: " + String(PITCH_PID_KD) + " m: " + String(PITCH_PID_MIN) + " M: " + String(PITCH_PID_MAX);
//        Serial.println(output);
//        Serial.println("Yaw");
//        output = "P: " + String(YAW_PID_KP) + " I: " + String(YAW_PID_KI) + " D: " + String(YAW_PID_KD) + " m: " + String(YAW_PID_MIN) + " M: " + String(YAW_PID_MAX);
//        Serial.println(output);
//      }
//      
      
    bluetoothString = "";
    receivedAll = false;
}



void getBluetoothData(){
   if (Serial.available()) {
    while(Serial.available()){
      mpu.resetFIFO();
      bluetoothChar = Serial.read();
//      if (!receivedAll)
      bluetoothString += bluetoothChar; 
      
      if(bluetoothChar == '|'){processString();receivedAll = true;}
      else {receivedAll = false;}
    }
  
  }
   
}

void getPIDValues(){
//  setY = 0;
//  setP = 1.48;
//  setR = 0.26;
  
//  PIDyaw_val= (int)PIDyaw.Compute((float)setY-yprdegree[0]);
  PIDyaw_val = (int)PIDyaw.Compute(setY-digitalSmooth(yprdegree[0],yawSmoothArray));
  PIDpitch_val= (int)PIDpitch.Compute(setP-digitalSmooth(yprdegree[1],pitchSmoothArray));
  PIDroll_val= (int)PIDroll.Compute(setR-digitalSmooth(yprdegree[2],rollSmoothArray));

}

void adjustMotors(){
  int m1_val=throttle+PIDroll_val+PIDpitch_val+PIDyaw_val;
  int m2_val=throttle-PIDroll_val+PIDpitch_val-PIDyaw_val;
//  int m2_val=throttle+PIDroll_val-PIDpitch_val-PIDyaw_val;
  int m3_val=throttle-PIDroll_val-PIDpitch_val+PIDyaw_val;
  int m4_val=throttle+PIDroll_val-PIDpitch_val-PIDyaw_val;
//  int m4_val=throttle-PIDroll_val+PIDpitch_val-PIDyaw_val;

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

  if(debug){
  Serial.print("Y2: ");
  Serial.print(yprdegree[0]);
  Serial.print(", P: ");
  Serial.print(yprdegree[1]);
  Serial.print(", R: ");
  Serial.print(yprdegree[2]);
  Serial.print(", M1: ");
  Serial.print(m1_val);
  Serial.print(", M2: ");
  Serial.print(m2_val);
  Serial.print(", M3: ");
  Serial.print(m3_val);
  Serial.print(", M4: ");
  Serial.print(m4_val);
//  Serial.print(", YPID: ");
//  Serial.print(PIDyaw_val);
//  Serial.print(", PPID: ");
//  Serial.print(PIDpitch_val);
//  Serial.print(", RPID: ");
//  Serial.println(PIDroll_val);
  Serial.println(" setY: " + String(setY) + " setP: " + String(setP) + " setR: " + String(setR));

}

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
//    do {
//        #ifdef BMP
//      P = getPressure();
//      a = pressure.altitude(P,baseline);
//        Serial.print("relative altitude: ");
//        if (a >= 0.0) Serial.print(" "); // add a space for positive numbers
//        if (a <= 0.0) a = 0;
//        Serial.print(a,1);
//        Serial.print(" meters, ");
//        if (a >= 0.0) Serial.print(" "); // add a space for positive numbers
//        Serial.print(a*3.28084,0);
//        Serial.println(" feet");
//      #endif
//    }
//    while (!mpuInterrupt && fifoCount < packetSize);
if (mpuInterrupt || mpu.getFIFOCount() > 400){
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
  
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount >= 1024) {
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
//
//            if (debug){
//              Serial.print("ypr\t");
//              Serial.print(ypr[0] * 180/M_PI);
//              Serial.print("\t");
//              Serial.print(ypr[1] * 180/M_PI);
//              Serial.print("\t");
//              Serial.println(ypr[2] * 180/M_PI);
//            }
            
            
        
    }
    }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================


void loop(){
  updateSensors();
  getBluetoothData();
  getPIDValues();
  adjustMotors();
}
