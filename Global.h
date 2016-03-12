//#define debug

/////////////////
//PID VARIABLES//
/////////////////

double ROLL_PID_KP = 0.450;
double ROLL_PID_KI = 0.950;
double ROLL_PID_KD = 0.011;
//double ROLL_PID_KI = 0;
//double ROLL_PID_KD = 0;
double ROLL_PID_MIN = -50.0;
double ROLL_PID_MAX = 50.0;

double PITCH_PID_KP =  0.450;
double PITCH_PID_KI = 0.950;
double PITCH_PID_KD = 0.011;
//double PITCH_PID_KI = 0;
//double PITCH_PID_KD = 0;
double PITCH_PID_MIN = -50.0;
double PITCH_PID_MAX = 50.0;

double YAW_PID_KP = 0.680;
double YAW_PID_KI =  0.500;
double YAW_PID_KD = 0.0001;
double YAW_PID_MIN = -50.0;
double YAW_PID_MAX = 50.0;

//Define the PID class
PIDCont PIDroll;
PIDCont PIDpitch;
PIDCont PIDyaw;
PIDCont PIDalt;
double altitudeHold = 1; //1 Meter

//Returned PID values
int PIDroll_val;
int PIDpitch_val;
int PIDyaw_val;
int PIDalt_val;

//Values to try to achieve
int setY = 0;
int setP = 0;
int setR = 0;

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
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yprdegree[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//Variables to smooth data from MPU6050
#define filterSamples   13              // filterSamples should  be an odd number, no smaller than 3
float yawSmoothArray [filterSamples];   // array for holding raw sensor values for yaw 
float pitchSmoothArray [filterSamples];   // array for holding raw sensor values for yaw 
float rollSmoothArray [filterSamples];   // array for holding raw sensor values for yaw 

//////////
//BMP180//
//////////

SFE_BMP180 pressure;
double baseline; // baseline pressure

///////////////////////
//BLUETOOTH VARIABLES//
///////////////////////

String bluetoothString;
int bluetoothInt;
char bluetoothChar;

int pIndex = -1;
int iIndex = -1;
int dIndex = -1;
int endIndex = -1;
int tIndex = -1;

///////////////////
//MOTOR VARIABLES//
///////////////////

int throttle;
int m1_val;
int m2_val;
int m3_val;
int m4_val;

//Motor Pins
#define MOTOR1 3
#define MOTOR2 9
#define MOTOR3 10
#define MOTOR4 11

//Motor PWM Levels
#define MAX_SIGNAL 765
//Minimum Value before motors begin spinning
#define MOTOR_RUN_LEVEL 630
//Value at which motors are not moving
#define MOTOR_ZERO_LEVEL 620

/////////////////
//LED VARIABLES//
/////////////////

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)


