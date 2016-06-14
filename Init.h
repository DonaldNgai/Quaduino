
void PID_init(){
  Serial.println("Roll");
  String output = "P: " + String(ROLL_PID_KP) + " I: " + String(ROLL_PID_KI) + " D: " + String(ROLL_PID_KD) + " m: " + String(ROLL_PID_MIN) + " M: " + String(ROLL_PID_MAX);
  Serial.println(output);
  Serial.println("Pitch");
  output = "P: " + String(PITCH_PID_KP) + " I: " + String(PITCH_PID_KI) + " D: " + String(PITCH_PID_KD) + " m: " + String(PITCH_PID_MIN) + " M: " + String(PITCH_PID_MAX);
  Serial.println(output);
  Serial.println("Yaw");
  output = "P: " + String(YAW_PID_KP) + " I: " + String(YAW_PID_KI) + " D: " + String(YAW_PID_KD) + " m: " + String(YAW_PID_MIN) + " M: " + String(YAW_PID_MAX);
  Serial.println(output);

    //                          Kp,        Ki,         Kd           Lval         Hval
  PIDroll.ChangeParameters(ROLL_PID_KP,ROLL_PID_KI,ROLL_PID_KD,ROLL_PID_MIN,ROLL_PID_MAX);
  PIDpitch.ChangeParameters(PITCH_PID_KP,PITCH_PID_KI,PITCH_PID_KD,PITCH_PID_MIN,PITCH_PID_MAX);
  PIDyaw.ChangeParameters(YAW_PID_KP,YAW_PID_KI,YAW_PID_KD,YAW_PID_MIN,YAW_PID_MAX);
//  PIDalt.ChangeParameters(ALT_PID_KP,ALT_PID_KI,ALT_PID_KD,ALT_PID_MIN,ALT_PID_MAX);
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
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
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
    
   // wait for ready
    Serial.println(F("\nSend any character to initialize DMP"));
     while (Serial.available() && Serial.read()); // empty buffer
     while (!Serial.available());                 // wait for data
     while (Serial.available() && Serial.read()){// empty buffer again
    } 

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

//Your offsets:  1773  869 1049  26  -34 26

//For my own chip
    mpu.setXGyroOffset(44);
    mpu.setYGyroOffset(-30);
    mpu.setZGyroOffset(19);
    mpu.setXAccelOffset(1814);
    mpu.setYAccelOffset(883);
    mpu.setZAccelOffset(1099);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println(F("Waiting for DMP to stablilize"));

        delay(20000);
        Serial.println(F("*********************************************************************************"));

        setY = yprdegree[0];
        setP = yprdegree[1];
        setR = yprdegree[2];
        prevY = setY;
        prevP = setP;
        prevR = setR;
        
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void initBMP(){
  if (pressure.begin())
    Serial.println("BMP180 init success");
    else
    {
      // Oops, something went wrong, this is usually a connection problem,
      // see the comments at the top of this sketch for the proper connections.
  
      Serial.println("BMP180 init fail (disconnected?)\n\n");
      while(1); // Pause forever.
    }

    baseline = getPressure();
    
    Serial.print("baseline pressure: ");
    Serial.print(baseline);
    Serial.println(" mb");  
}

void initMotors(){
   
//  pinMode(MOTOR1,OUTPUT);
//  pinMode(MOTOR2,OUTPUT);
//  pinMode(MOTOR3,OUTPUT);
//  pinMode(MOTOR4,OUTPUT);
  MOTOR1.attach(MOTOR1PIN);
  MOTOR2.attach(MOTOR2PIN);
  MOTOR3.attach(MOTOR3PIN);
  MOTOR4.attach(MOTOR4PIN);
  //Arm motor
  throttle = map(0,0,100,0,179);

//  analogWrite(MOTOR1,throttle);
//  analogWrite(MOTOR2,throttle);
//  analogWrite(MOTOR3,throttle);
//  analogWrite(MOTOR4,throttle);
  MOTOR1.write(throttle);
  MOTOR2.write(throttle);
  MOTOR3.write(throttle);
  MOTOR4.write(throttle);
  Serial.println("Now writing minimum output to arm motors.");
}



