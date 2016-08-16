#define SCALING_FACTOR 100

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define CRAZY_ANGLE_THRESHOLD 48 * SCALING_FACTOR
//#define debug
unsigned long timeOfLastSignal;

#define FAST_SAMPLE_TIME 1250 //400Hz using Micros() //1000 millisecond(1 sec) per 100 commands = 10 milliseconds per command
#define SLOW_SAMPLE_TIME 20 //50Hz using Millis()
unsigned long slowLoopTimer = 0;
unsigned long fastLoopTimer = 0;
unsigned int timeChange = 0;

//thirty seconds
#define FAILSAFE_THRESHOLD   30000
bool failSafe = false;
/////////////////
//PID VARIABLES//
/////////////////

double ROLL_PID_KP = 0.02;
//double ROLL_PID_KI = 0.950;
//double ROLL_PID_KD = 0.011;
double ROLL_PID_KI = 0;
double ROLL_PID_KD = 0;
double ROLL_PID_MIN = -100.0;
double ROLL_PID_MAX = 100.0;

double PITCH_PID_KP = 0.02;
//double PITCH_PID_KI = 0.950;
//double PITCH_PID_KD = 0.011;
double PITCH_PID_KI = 0;
double PITCH_PID_KD = 0;
double PITCH_PID_MIN = -100.0;
double PITCH_PID_MAX = 100.0;

double YAW_PID_KP = 0.0027;
double YAW_PID_KI =  0.0;
double YAW_PID_KD = 0.0;
//double YAW_PID_KP = 0.680;
//double YAW_PID_KI =  0.500;
//double YAW_PID_KD = 0.0001;
double YAW_PID_MIN = -100.0;
double YAW_PID_MAX = 100.0;

//Define the PID class
PIDCont PIDroll;
PIDCont PIDpitch;
PIDCont PIDyaw;

//Returned PID values
int PIDroll_val;
int PIDpitch_val;
int PIDyaw_val;

//Values to try to achieve
int setY = 0;
int setP = 0;
int setR = 0;
int prevY = 0;
int prevP = 0;
int prevR = 0;

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
int yprdegree[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
int gyroX;
int gyroY;
int gyroZ;

//Variables to smooth data from MPU6050
#define filterSamples   17              // filterSamples should  be an odd number, no smaller than 3
int yawSmoothArray [filterSamples];   // array for holding raw sensor values for yaw 
int pitchSmoothArray [filterSamples];   // array for holding raw sensor values for yaw 
int rollSmoothArray [filterSamples];   // array for holding raw sensor values for yaw 

int gyroXSmoothArray [filterSamples];   // array for holding raw sensor values for yaw 
int gyroYSmoothArray [filterSamples];   // array for holding raw sensor values for yaw 
int gyroZSmoothArray [filterSamples];   // array for holding raw sensor values for yaw 

#define bluetoothDataLength 70

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
#define MAX_SIGNAL 1460
//Minimum Value before motors begin spinning
#define MOTOR_RUN_LEVEL 580
//Value at which motors are not moving
#define MOTOR_ZERO_LEVEL 550

/////////////////
//LED VARIABLES//
/////////////////

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)



