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
//  Serial.println(String(endIndex) + " " + String(commaIndex) + " " + String(temp) + " " + String(bluetoothString));
}

void processString(){
//Failsafe,Debug,PID,Throttle,rateAndControl,PhoneYaw,PhonePitch,PhoneRoll,RP_P,RP_I,RP_D,Y_P,Y_I,Y_D|
//,0,0,0,0,0,0,0.25,0.0,0.0,0.0,-0.01,0.0|
  int bluetoothInt;
  
  updateIndexes();
  if (temp.charAt(0) == 'C')
  {
    if (bluetoothString.length() == temp.substring(1,temp.length()).toInt()){
    }
    else{
      bluetoothString = "";
      return;
    }
  }
  else
  {
    return;
  }

  if (
    bluetoothString.substring(bluetoothString.length()-1,bluetoothString.length()) == "|"
    )
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

    //Calibrate
//    updateIndexes();
//    if (temp == "1") {
////        failSafe = false;
//        setY = yprdegree[0];
//        setP = yprdegree[1];
//        setR = yprdegree[2];
//        prevY = setY;
//        prevP = setP;
//        prevR = setR;
//    }
//    else if (control == false){
//        setY = prevY;
//        setP = prevP;
//        setR = prevR;
//    }

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
//        if (bluetoothInt == 0){Serial.println("Z");}
        throttle = map(bluetoothInt,0,100,MOTOR_ZERO_LEVEL,MAX_SIGNAL);

      }
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

//    setY=map(receivedYaw,YAW_RMIN,YAW_RMAX,YAW_WMAX,YAW_WMIN);
setY = receivedYaw;
//    if (rate == false){
//      setR=map(receivedRoll,ROLL_RMIN,ROLL_RMAX,ROLL_WMIN,ROLL_WMAX);
//      setP=map(receivedPitch,PITCH_RMIN,PITCH_RMAX,PITCH_WMAX,PITCH_WMIN);
setR = receivedRoll;
setP = receivedPitch;
//    }
//    else{
//      setR=map(receivedRoll,ROLL_RMIN,ROLL_RMAX,ROLL_WMIN*RX_RATE_SENSITIVITY,ROLL_WMAX*RX_RATE_SENSITIVITY);
//      setP=map(receivedPitch,PITCH_RMIN,PITCH_RMAX,PITCH_WMAX*RX_RATE_SENSITIVITY,PITCH_WMIN*RX_RATE_SENSITIVITY); 
//    }


    char floatbuf[32]; // make this at least big enough for the whole string
    double toChange = 0.0;

    //Roll and Pitch PID
    updateIndexes();
    if(changePID){
      temp.toCharArray(floatbuf, sizeof(floatbuf));
      toChange = atof(floatbuf);
      ROLL_PID_KP = toChange;
      PITCH_PID_KP = toChange;
//      ANGLEX_KP = toChange;
//      ANGLEY_KP = toChange;
    }
    updateIndexes();
    if(changePID){
      temp.toCharArray(floatbuf, sizeof(floatbuf));
      toChange = atof(floatbuf);
      ROLL_PID_KI = toChange;
      PITCH_PID_KI = toChange;
//      ANGLEX_KI = toChange;
//      ANGLEY_KI = toChange;
    }
    updateIndexes();
    if(changePID){
      temp.toCharArray(floatbuf, sizeof(floatbuf));
      toChange = atof(floatbuf);
      ROLL_PID_KD = toChange;
      PITCH_PID_KD = toChange;
//      ANGLEX_KD = toChange;
//      ANGLEY_KD = toChange;
    }
    //YAW PID
    updateIndexes();
    if(changePID){
      temp.toCharArray(floatbuf, sizeof(floatbuf));
      toChange = atof(floatbuf);
      YAW_PID_KP = toChange;
    }
    updateIndexes();
    if(changePID){
      temp.toCharArray(floatbuf, sizeof(floatbuf));
      toChange = atof(floatbuf);
      YAW_PID_KI = toChange;
    }
    updateIndexes();
    if(changePID){
      temp.toCharArray(floatbuf, sizeof(floatbuf));
      toChange = atof(floatbuf);
      YAW_PID_KD = toChange;
    }

    //SET PID
    //                  //                          Kp,        Ki,         Kd           Lval         Hval
    if (changePID){
      
      PIDroll.resetI();
      PIDpitch.resetI();
      PIDyaw.resetI();
      PIDangleX.resetI();
      PIDangleY.resetI();
      PIDroll.ChangeParameters(ROLL_PID_KP,ROLL_PID_KI,ROLL_PID_KD,ROLL_PID_MIN,ROLL_PID_MAX);
      PIDpitch.ChangeParameters(PITCH_PID_KP,PITCH_PID_KI,PITCH_PID_KD,PITCH_PID_MIN,PITCH_PID_MAX);
      PIDyaw.ChangeParameters(YAW_PID_KP,YAW_PID_KI,YAW_PID_KD,YAW_PID_MIN,YAW_PID_MAX);
//      PIDangleX.ChangeParameters(ANGLEX_KP,ANGLEX_KI,ANGLEX_KD,ANGLEX_MIN,ANGLEX_MAX);
//      PIDangleY.ChangeParameters(ANGLEY_KP,ANGLEY_KI,ANGLEY_KD,ANGLEY_MIN,ANGLEY_MAX);
//      
//      Serial.println(F("ARoll"));
//      Serial.println("P: " + String(ANGLEX_KP) + " I: " + String(ANGLEX_KI) + " D: " + String(ANGLEX_KD) + " m: " + String(ANGLEX_MIN) + " M: " + String(ANGLEX_MAX));
//      Serial.println(F("APitch"));
//      Serial.println("P: " + String(ANGLEY_KP) + " I: " + String(ANGLEY_KI) + " D: " + String(ANGLEY_KD) + " m: " + String(ANGLEY_MIN) + " M: " + String(ANGLEY_MAX));

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
//    Serial.println(Serial.available());

    int upper = min(bluetoothDataLength,Serial.available());
    for (int byteCounter = 0 ; byteCounter < upper ; byteCounter ++){
//    while(Serial.available() && byteCounter < bluetoothDataLength)
//    {
//      byteCounter++;
//      unsigned long tm = micros();
      bluetoothChar = Serial.read();
//      tm = micros() - tm;
//      Serial.print("a");
//      Serial.println(tm);
      
      bluetoothString += bluetoothChar; 
      
      if(bluetoothChar == '|')
      {
//        tm = micros();
//        if (millis() < 40000){
        processString();
//        }
//        else{
//          Serial.println("Stopped");
//        }
//        tm = micros() - tm;
//        Serial.print("b");
//        Serial.println(tm);

//        tm = micros();
        
        while(Serial.available()){
          Serial.read();
        }
        
//        tm = micros() - tm;
//        Serial.print("c");
//        Serial.println(tm);
        
        break;
      }
    }
  }
}

//int smoothY,smoothP,smoothR;
//int YinIndex,PinIndex,RinIndex;
//
//void updateOrientationData() {
//    Quaternion q;           // [w, x, y, z]         quaternion container
//    VectorFloat gravity;    // [x, y, z]            gravity vector
//    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//    // if programming failed, don't try to do anything
//    if (!dmpReady) return;
//    
//  // get current FIFO count
//  fifoCount = mpu.getFIFOCount();
//  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
//  if (mpu.getFIFOCount() >= packetSize)
//  {
//    // reset interrupt flag and get INT_STATUS byte
//    mpuInterrupt = false;
//    mpuIntStatus = mpu.getIntStatus();
//
//    fifoCount = mpu.getFIFOCount();
//  
//    // check for overflow (this should never happen unless our code is too inefficient)
//      if ((mpuIntStatus & 0x10) || fifoCount >= 1024) 
//        {
//          // reset so we can continue cleanly
//          mpu.resetFIFO();
//          Serial.println(F("FIFO overflow!"));
//  
//      // otherwise, check for DMP data ready interrupt (this should happen frequently)
//        } 
//      else if (mpuIntStatus & 0x02) 
//        {
//        // wait for correct available data length, should be a VERY short wait
//        
//
//        // Remove all the old data - maybe need to do -1
//        mpu.getFIFOBytes(fifoBuffer, fifoCount - packetSize);
//        // read a packet from FIFO
//        mpu.getFIFOBytes(fifoBuffer, packetSize);
//        mpu.resetFIFO();
//        // track FIFO count here in case there is > 1 packet available
//        // (this lets us immediately read more without waiting for an interrupt)
////        fifoCount -= packetSize;
//        
//            // display Euler angles in degrees
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetGravity(&gravity, &q);
//            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            yprdegree[0] = int((ypr[0] * 180/M_PI)*SCALING_FACTOR);
//            yprdegree[1] = int((ypr[1] * 180/M_PI)*SCALING_FACTOR);
//            yprdegree[2] = int((ypr[2] * 180/M_PI)*SCALING_FACTOR);
//            Serial.println(" Y: " + String(yprdegree[0]) + " smP: " + String(yprdegree[1]) + " smR: " + String(yprdegree[2]));
//
//            smoothY = digitalSmooth(yprdegree[0],yawSmoothArray,yawsortedArray,YinIndex,filterSamples);
//            smoothP = digitalSmooth(yprdegree[1],pitchSmoothArray,pitchsortedArray,PinIndex,filterSamples);
//            smoothR = digitalSmooth(yprdegree[2],rollSmoothArray,rollsortedArray,RinIndex,filterSamples);
//              Serial.println(" smY: " + String(smoothY) + " smP: " + String(smoothP) + " smR: " + String(smoothR));
//
//            //failsafe while debugging
//            if (abs(smoothP) + abs(smoothR) > CRAZY_ANGLE_THRESHOLD){
//              failSafe = true;
//              Serial.println(" smY: " + String(smoothY) + " smP: " + String(smoothP) + " smR: " + String(smoothR));
//              Serial.println(F("Crazy Angle"));
//            }
//            
////            YinIndex = (YinIndex + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
////            PinIndex = (PinIndex + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
////            RinIndex = (RinIndex + 1) % filterSamples;    // increment counter and roll over if necc. -  % (modulo operator) rolls over variable
//                        
//        }
//    }
//}

void getPIDValues(){
  
  if (rateAndControl == true){ 
//    if (debug && millis() - printTimer > PRINT_TIME)
//    Serial.print("h");
//    Serial.print(" recP: " + String(receivedPitch) + " recR: " + String(receivedRoll));
//    Serial.print("P:" + String(angles[0]) + " R:" + String(angles[1]));
//    setP=(int)PIDangleY.Compute(setP+smoothP,gy_aver,setP/RX_ANGLE_DAMPNING); 
//    setR=(int)PIDangleX.Compute(setR-smoothR,gx_aver,setR/RX_ANGLE_DAMPNING); 
    setP=(int)PIDangleY.Compute(receivedPitch-angles[0]); 
    setR=(int)PIDangleX.Compute(receivedRoll+angles[1]); 
//    Serial.println(" setP: " + String(setP) + " setR: " + String(setR));
//    setP=(int)PIDangleY.Compute(setP+angles[0],gy_aver,setP/RX_ANGLE_DAMPNING); 
//    setR=(int)PIDangleX.Compute(setR-angles[1],gx_aver,setR/RX_ANGLE_DAMPNING); 
  } 
  
  PIDroll_val= (int)PIDroll.Compute((double)(setR-gy_aver)); 
  PIDpitch_val= (int)PIDpitch.Compute((double)(setP-gx_aver)); 
  PIDyaw_val= (int)PIDyaw.Compute((double)(wrap_180(setY-gz_aver)));

//  Serial.print("G");
//  Serial.print("setY:" + String(setY) + "setP: " + String(setP) + "setR: " + String(setR));
//  Serial.print("smGY:" + String(gz_aver) + "smGP:" + String(gx_aver) + "smGR:" + String(gy_aver));
//  Serial.println("PIDY:" + String(PIDyaw_val) + "PIDP:" + String(PIDpitch_val) + "PIDR:" + String(PIDroll_val));

  //  PIDyaw_val = (int)PIDyaw.Compute(((((int)((smoothY - setY) + 180) % 360) + 360) % 360)-180);
    //To prevent extremely small PID values
  //  PIDyaw_val = (int)PIDyaw.Compute(wrap_180((smoothY-setY)/SCALING_FACTOR));
  //  PIDpitch_val= (int)PIDpitch.Compute((setP-smoothP)/SCALING_FACTOR);
  //  PIDroll_val= (int)PIDroll.Compute((setR-smoothR)/SCALING_FACTOR);

}

void adjustMotors(){
  
  
  m1_val =throttle-PIDroll_val+PIDpitch_val+PIDyaw_val;
//  m2_val=throttle-PIDroll_val+PIDpitch_val-PIDyaw_val;
  m2_val=throttle-PIDroll_val-PIDpitch_val-PIDyaw_val;
  m3_val=throttle+PIDroll_val-PIDpitch_val+PIDyaw_val;
//  m4_val=throttle+PIDroll_val-PIDpitch_val-PIDyaw_val;
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
  if (debug && millis() - printTimer > PRINT_TIME){
    //  Serial.print(", YPID: " + String(PIDyaw_val) + ", PPID: " + String(PIDpitch_val) + ", RPID: " + String(PIDroll_val) );
    //  Serial.println(" setY: " + String(((((int)((smoothY - setY) + 180) % 360) + 360) % 360)-180) + " setP: " + String(setP-smoothP) + " setR: " + String(setR-smoothR));
    
      Serial.print("M1:" + String(m1_val) + "M2:" + String(m2_val) + "M3:" + String(m3_val) + "M4:" + String(m4_val));
//      Serial.print("setY:" + String(setY) + "setP: " + String(setP) + "setR: " + String(setR));
      Serial.print(" recY: " + String(receivedYaw) + " recP: " + String(receivedPitch) + " recR: " + String(receivedRoll));

      Serial.print("P:" + String(angles[0]) + " R:" + String(angles[1]));

      Serial.print("smGY:" + String(gz_aver) + "smGP:" + String(gx_aver) + "smGR:" + String(gy_aver));
//
//      Serial.print("PIDY:" + String(PIDyaw_val) + "PIDP:" + String(PIDpitch_val) + "PIDR:" + String(PIDroll_val));

      //Needed so that the phone knows to reset throttle when failsafe is triggered by drone
//      Serial.print(" F:" + String(failSafe));
      Serial.println("");
      printTimer = millis();
    }
}
//unsigned long tm = micros();
void loop(){
  //failsafe
  if (millis() - timeOfLastSignal > FAILSAFE_THRESHOLD && !failSafe){
    failSafe = true;
    Serial.println(F("No Signal"));
  }
  
//  unsigned long temp = 0;
    //1-6 millis
  if (millis()-slowLoopTimer >= SLOW_SAMPLE_TIME){
//    Serial.println("HIINHIHIHIHIHIHI");

//  unsigned long tm = micros();
    getBluetoothData();
    
//    tm = micros() - tm;
//    Serial.print("g");
//    Serial.println(tm);
    
    if (failSafe) {Serial.println(F("FAIL!"));}
    
//    updateOrientationData();
    updateAcc();
    slowLoopTimer = millis();
  }

  if (micros()-fastLoopTimer >= FAST_SAMPLE_TIME){
    //400 Micro, Max 512
    getPIDValues();
    //36 Micro, Max 64
//    tm = micros() - tm;
//    if (tm > 3000)
//    Serial.println(tm);
    
    adjustMotors();
//    tm = micros();
    updateGyroData();
    updateSensorVal();
    fastLoopTimer = micros();
  }
  

//  Serial.print(F("T5: "));
//  Serial.print(micros()-lastTime);
//  Serial.println(F(""));
  //make sure to wait until 50Hz
//  timeChange = (micros() - lastTime);
//  while (timeChange < SAMPLE_TIME)
//  {
//    timeChange = (micros() - lastTime);
//  }
}
