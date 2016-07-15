//#define debug
unsigned long timeOfLastSignal;

#define SAMPLE_TIME 13 //1000 millisecond(1 sec) per 100 commands = 10 milliseconds per command
unsigned long lastTime = millis()-SAMPLE_TIME;
unsigned int timeChange = 0;

//thirty seconds
#define FAILSAFE_THRESHOLD   30000
bool failSafe = false;
/////////////////
//PID VARIABLES//
/////////////////

double ROLL_PID_KP = 0.145;
//double ROLL_PID_KI = 0.950;
//double ROLL_PID_KD = 0.011;
double ROLL_PID_KI = 0;
double ROLL_PID_KD = 0;
double ROLL_PID_MIN = -15.0;
double ROLL_PID_MAX = 15.0;

double PITCH_PID_KP = 0.145;
//double PITCH_PID_KI = 0.950;
//double PITCH_PID_KD = 0.011;
double PITCH_PID_KI = 0;
double PITCH_PID_KD = 0;
double PITCH_PID_MIN = -15.0;
double PITCH_PID_MAX = 15.0;

double YAW_PID_KP = 0.027;
double YAW_PID_KI =  0.0;
double YAW_PID_KD = 0.0;
//double YAW_PID_KP = 0.680;
//double YAW_PID_KI =  0.500;
//double YAW_PID_KD = 0.0001;
double YAW_PID_MIN = -15.0;
double YAW_PID_MAX = 15.0;

//Define the PID class
PIDCont PIDroll;
PIDCont PIDpitch;
PIDCont PIDyaw;

//Returned PID values
double PIDroll_val;
double PIDpitch_val;
double PIDyaw_val;

//Values to try to achieve
float setY = 0;
float setP = 0;
float setR = 0;
float prevY = 0;
float prevP = 0;
float prevR = 0;

///////////
//MPU6050//
///////////

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
float yprdegree[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//Variables to smooth data from MPU6050
#define filterSamples   9              // filterSamples should  be an odd number, no smaller than 3
float yawSmoothArray [filterSamples];   // array for holding raw sensor values for yaw 
float pitchSmoothArray [filterSamples];   // array for holding raw sensor values for yaw 
float rollSmoothArray [filterSamples];   // array for holding raw sensor values for yaw 

//////////
//BMP180//
//////////

//SFE_BMP180 pressure;
//double baseline; // baseline pressure

///////////////////////
//BLUETOOTH VARIABLES//
///////////////////////

String bluetoothString;
int bluetoothInt;
char bluetoothChar;

int endIndex = -1;

///////////////////
//MOTOR VARIABLES//
///////////////////

int throttle;
int m1_val;
int m2_val;
int m3_val;
int m4_val;

//Motor Pins
#define MOTOR1PIN 3
#define MOTOR2PIN 9
#define MOTOR3PIN 10
#define MOTOR4PIN 11
Servo MOTOR1;
Servo MOTOR2;
Servo MOTOR3;
Servo MOTOR4;

//Motor PWM Levels
//#define MAX_SIGNAL 765
////Minimum Value before motors begin spinning
//#define MOTOR_RUN_LEVEL 630
////Value at which motors are not moving
//#define MOTOR_ZERO_LEVEL 620
#define MAX_SIGNAL 90
//Minimum Value before motors begin spinning
#define MOTOR_RUN_LEVEL 4
//Value at which motors are not moving
#define MOTOR_ZERO_LEVEL 0

/////////////////
//LED VARIABLES//
/////////////////

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)



