
void PID_init(){
    //                          Kp,        Ki,         Kd           Lval         Hval
  PIDroll.ChangeParameters(ROLL_PID_KP,ROLL_PID_KI,ROLL_PID_KD,ROLL_PID_MIN,ROLL_PID_MAX);
  PIDpitch.ChangeParameters(PITCH_PID_KP,PITCH_PID_KI,PITCH_PID_KD,PITCH_PID_MIN,PITCH_PID_MAX);
  PIDyaw.ChangeParameters(YAW_PID_KP,YAW_PID_KI,YAW_PID_KD,YAW_PID_MIN,YAW_PID_MAX);
//  PIDalt.ChangeParameters(ALT_PID_KP,ALT_PID_KI,ALT_PID_KD,ALT_PID_MIN,ALT_PID_MAX);
}

void setPIDValues(){
  bluetoothString = "";
  digitalWrite(LED_PIN,HIGH);
  bool done = false;
  while(!done){
    while (Serial.available()){// empty buffer again
      delay(3);
      bluetoothChar = Serial.read();
      bluetoothString += bluetoothChar;
    } 
    if (bluetoothString != ""){
      while(bluetoothString != "")
      {
        String sub;
        int endIndex;
        endIndex = bluetoothString.indexOf("|");
        sub = bluetoothString.substring(0,endIndex);
        Serial.println(sub);
        bluetoothString.remove(0,endIndex+1);
        endIndex = sub.indexOf(":");
        if (sub == "YawP"){YAW_PID_KP = sub.substring(endIndex+1).toInt();}
        if (sub == "YawI"){YAW_PID_KI = sub.substring(endIndex+1).toInt();}
        if (sub == "YawD"){YAW_PID_KD = sub.substring(endIndex+1).toInt();}
        if (sub == "PitchP"){PITCH_PID_KP = sub.substring(endIndex+1).toInt();}
        if (sub == "PitchI"){PITCH_PID_KI = sub.substring(endIndex+1).toInt();}
        if (sub == "PitchD"){PITCH_PID_KD = sub.substring(endIndex+1).toInt();}
        if (sub == "RollP"){ROLL_PID_KP = sub.substring(endIndex+1).toInt();}
        if (sub == "RollI"){ROLL_PID_KI = sub.substring(endIndex+1).toInt();}
        if (sub == "RollD"){ROLL_PID_KD = sub.substring(endIndex+1).toInt();}
      }
      done = true;
      digitalWrite(LED_PIN,LOW);
    }
    
  }
  bluetoothString = "";
}

void initCommunication()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
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
     while (Serial.available()){// empty buffer again
      delay(2);
      bluetoothChar = Serial.read();
      bluetoothString += bluetoothChar;
    } 
     
    if (bluetoothString == "set"){
      setPIDValues();
    }
    bluetoothString = "";

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

     // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

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
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
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
   //Arm motor
  pinMode(MOTOR1,OUTPUT);
  pinMode(MOTOR2,OUTPUT);
  pinMode(MOTOR3,OUTPUT);
  pinMode(MOTOR4,OUTPUT);
  throttle = map(0,0,685,620,765);

  analogWrite(MOTOR1,throttle);
  analogWrite(MOTOR2,throttle);
  analogWrite(MOTOR3,throttle);
  analogWrite(MOTOR4,throttle);
  Serial.println("Now writing minimum output to arm motors.");
}



