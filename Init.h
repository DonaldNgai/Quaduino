
void PID_init(){
    //                          Kp,        Ki,         Kd           Lval         Hval
  PIDroll.ChangeParameters(ROLL_PID_KP,ROLL_PID_KI,ROLL_PID_KD,ROLL_PID_MIN,ROLL_PID_MAX);
  PIDpitch.ChangeParameters(PITCH_PID_KP,PITCH_PID_KI,PITCH_PID_KD,PITCH_PID_MIN,PITCH_PID_MAX);
  PIDyaw.ChangeParameters(YAW_PID_KP,YAW_PID_KI,YAW_PID_KD,YAW_PID_MIN,YAW_PID_MAX);
  PIDangleX.ChangeParameters(ANGLEX_KP,ANGLEX_KI,ANGLEX_KD,ANGLEX_MIN,ANGLEX_MAX);
  PIDangleY.ChangeParameters(ANGLEY_KP,ANGLEY_KI,ANGLEY_KD,ANGLEY_MIN,ANGLEY_MAX); 
}

void initCommunication()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        
        TWBR = 12; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
}


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void initMPU(){
    Serial.println(F("Initializing MPU6050"));
    mpu.initialize();

        // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    //Your offsets:  1832 880 1041 42 -33 19

    //For my own chip
    mpu.setXGyroOffset(GYRO_X_OFFSET);
    mpu.setYGyroOffset(GYRO_Y_OFFSET);
    mpu.setZGyroOffset(GYRO_Z_OFFSET);
    mpu.setXAccelOffset(ACC_X_OFFSET);
    mpu.setYAccelOffset(ACC_Y_OFFSET);
    mpu.setZAccelOffset(ACC_Z_OFFSET);

    // make sure it worked (returns 0 if so)
    if (mpu.testConnection()) {

        Serial.println(F("Roll"));
        Serial.println("P: " + String(ROLL_PID_KP) + " I: " + String(ROLL_PID_KI) + " D: " + String(ROLL_PID_KD) + " m: " + String(ROLL_PID_MIN) + " M: " + String(ROLL_PID_MAX));
        Serial.println(F("Pitch"));
        Serial.println("P: " + String(PITCH_PID_KP) + " I: " + String(PITCH_PID_KI) + " D: " + String(PITCH_PID_KD) + " m: " + String(PITCH_PID_MIN) + " M: " + String(PITCH_PID_MAX));
        Serial.println(F("Yaw"));
        Serial.println("P: " + String(YAW_PID_KP) + " I: " + String(YAW_PID_KI) + " D: " + String(YAW_PID_KD) + " m: " + String(YAW_PID_MIN) + " M: " + String(YAW_PID_MAX));

        delay(MPU_STABLE_DELAY);
        Serial.println(F("*********************************************************************************"));

        setY = yprdegree[0];
        setP = yprdegree[1];
        setR = yprdegree[2];
        prevY = setY;
        prevP = setP;
        prevR = setR;
        
    } 
}

void initMotors(){
  
  MOTOR1.attach(MOTOR1PIN);
  MOTOR2.attach(MOTOR2PIN);
  MOTOR3.attach(MOTOR3PIN);
  MOTOR4.attach(MOTOR4PIN);
  
  //Arm motor
  throttle = map(0,0,100,MOTOR_ZERO_LEVEL,MAX_SIGNAL);

  MOTOR1.writeMicroseconds(throttle);
  MOTOR2.writeMicroseconds(throttle);
  MOTOR3.writeMicroseconds(throttle);
  MOTOR4.writeMicroseconds(throttle);
  Serial.println("Now writing minimum output to arm motors.");
}



