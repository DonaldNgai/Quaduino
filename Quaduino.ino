// I2Cdev, MPU6050 and BMP180 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
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

}

// ===================================================================
// ===                    SUPPORTING FUNCTIONS                     ===
// ===================================================================

bool control = false;
bool changePID = false;

String temp = "";

void updateIndexes(){
  int commaIndex = -1;

  endIndex = bluetoothString.length();
  commaIndex = bluetoothString.indexOf(',');
  temp = bluetoothString.substring(0,commaIndex);  
  bluetoothString = bluetoothString.substring(commaIndex + 1,endIndex);
//  Serial.println(String(endIndex) + " " + String(commaIndex) + " " + String(temp) + " " + String(bluetoothString));
}

void processString(){
//Failsafe,Calibrate,PID,Throttle,Control,PhoneYaw,PhonePitch,PhoneRoll,RP_P,RP_I,RP_D,Y_P,Y_I,Y_D|
//,0,0,0,0,0,0,0.25,0.0,0.0,0.0,-0.01,0.0|
  int bluetoothInt;
  
  //checksum
  bool goodCheck = false;
  updateIndexes();
  if (temp.charAt(0) == 'C')
  {
    if (bluetoothString.length() == temp.substring(1,temp.length()).toInt()){
      goodCheck = true;
    }
  }

  if (goodCheck && 
    bluetoothString.substring(bluetoothString.length()-1,bluetoothString.length()) == "|"
    )
  {

    timeOfLastSignal = millis();
    //Failsafe
    updateIndexes();
    //Flag to turn off failsafe
    if (temp == "1") failSafe = false;
//    else failSafe = false;

    //Calibrate
    updateIndexes();
    if (temp == "1") {
//        failSafe = false;
        setY = yprdegree[0];
        setP = yprdegree[1];
        setR = yprdegree[2];
        prevY = setY;
        prevP = setP;
        prevR = setR;
    }
    else if (control == false){
        setY = prevY;
        setP = prevP;
        setR = prevR;
    }

    //PID
    updateIndexes();
    if (temp == "1") changePID = true;
    else changePID = false;

    //Throttle
    updateIndexes();
    bluetoothInt = temp.toInt();
    if (failSafe){
      throttle = map(0,0,100,MOTOR_ZERO_LEVEL,MAX_SIGNAL);
    }
    else{
       if (bluetoothInt <= 100 && bluetoothInt >= 0){
//      throttle = map(bluetoothInt,0,100,620,765);
        throttle = map(bluetoothInt,0,100,MOTOR_ZERO_LEVEL,MAX_SIGNAL);

      }
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
    while(Serial.available())
    {
      bluetoothChar = Serial.read();
      
      bluetoothString += bluetoothChar; 
      
      if(bluetoothChar == '|')
      {
        processString();
        break;
      }
    }
  }
}

double smoothY,smoothP,smoothR;
int YinIndex,PinIndex,RinIndex;

void getPIDValues(){
  
  smoothY = digitalSmooth(yprdegree[0],yawSmoothArray,YinIndex);
  smoothP = digitalSmooth(yprdegree[1],pitchSmoothArray,PinIndex);
  smoothR = digitalSmooth(yprdegree[2],rollSmoothArray,RinIndex);
  //failsafe while debugging
  if (abs(smoothP)+ abs(smoothR) > 48){
    failSafe = true;
    Serial.println(F("Crazy Angle"));
  }
  YinIndex = (YinIndex + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  PinIndex = (PinIndex + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  RinIndex = (RinIndex + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
  PIDyaw_val = PIDyaw.Compute(((((int)((smoothY - setY) + 180) % 360) + 360) % 360)-180);
  
  PIDpitch_val= PIDpitch.Compute(setP-smoothP);
  PIDroll_val= PIDroll.Compute(setR-smoothR);

//  Serial.println("YPID: " + String(PIDyaw_val) + ", PPID: " + String(PIDpitch_val) + ", RPID: " + String(PIDroll_val) );

}

void adjustMotors(){
  double m1_val=throttle+PIDroll_val+PIDpitch_val+PIDyaw_val;
  double m2_val=throttle-PIDroll_val+PIDpitch_val-PIDyaw_val;
//  int m2_val=throttle+PIDroll_val-PIDpitch_val-PIDyaw_val;
  double m3_val=throttle-PIDroll_val-PIDpitch_val+PIDyaw_val;
  double m4_val=throttle+PIDroll_val-PIDpitch_val-PIDyaw_val;
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

// TODO i don't get it but putting in serial print makes the pid controller work but takig it out makes it not work

  Serial.print("M1: " + String(m1_val) + ", M2: " + String(m2_val) + ", M3: " + String(m3_val) + ", M4: " + String(m4_val));
//  Serial.print(", YPID: " + String(PIDyaw_val) + ", PPID: " + String(PIDpitch_val) + ", RPID: " + String(PIDroll_val) );
//  Serial.println(" setY: " + String(((((int)((smoothY - setY) + 180) % 360) + 360) % 360)-180) + " setP: " + String(setP-smoothP) + " setR: " + String(setR-smoothR));
  Serial.print(" setY: " + String(setY) + " setP: " + String(setP) + " setR: " + String(setR));
  Serial.print(" smY: " + String(smoothY) + " smP: " + String(smoothP) + " smR: " + String(smoothR));
  Serial.println(" F:" + String(failSafe));

//  }

    MOTOR1.writeMicroseconds(m1_val);
    MOTOR2.writeMicroseconds(m2_val);
    MOTOR3.writeMicroseconds(m3_val);
    MOTOR4.writeMicroseconds(m4_val);
//  analogWrite(MOTOR1,m1_val);
//  analogWrite(MOTOR2,m2_val);
//  analogWrite(MOTOR3,m3_val);
//  analogWrite(MOTOR4,m4_val);
}

void updateSensors() {
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
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
  if (mpu.getFIFOCount() >= packetSize)
  {
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
  
    // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount >= 1024) 
        {
          // reset so we can continue cleanly
          mpu.resetFIFO();
          Serial.println(F("FIFO overflow!"));
  
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } 
      else if (mpuIntStatus & 0x02) 
        {
        // wait for correct available data length, should be a VERY short wait
//        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.resetFIFO();
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
//        fifoCount -= packetSize;
        
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            yprdegree[0] = (ypr[0] * 180/M_PI);
            yprdegree[1] = (ypr[1] * 180/M_PI);
            yprdegree[2] = (ypr[2] * 180/M_PI);
            
        }
    }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop(){
  unsigned long temp = millis();

  //failsafe
  if (millis() - timeOfLastSignal > FAILSAFE_THRESHOLD){
    failSafe = true;
    Serial.println(F("No Signal"));
  }

  if (failSafe)
  Serial.println(F("FAIL!!!!!!!"));

  //1-6 millis
  getBluetoothData();

//  Serial.print(F("T5: "));
//  Serial.print(millis()-lastTime);
//  Serial.println(F(""));
  //make sure to wait until 50Hz
  timeChange = (millis() - lastTime);
  while (timeChange < SAMPLE_TIME)
  {
    timeChange = (millis() - lastTime);
  }

  //1-5 millis
  updateSensors();
  //1-3 millis
  getPIDValues();

  lastTime = millis();
  
  //5-12
  adjustMotors();

//10-18 millis
//  Serial.print(F("T5: "));
//  Serial.print(millis()-temp);
//  Serial.println(F(""));
}
