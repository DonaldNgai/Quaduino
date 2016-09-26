#define SCALING_FACTOR 100

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define CRAZY_ANGLE_THRESHOLD 48 * SCALING_FACTOR
#define MPU_STABLE_DELAY 20000//20000
//#define debug
unsigned long timeOfLastSignal;

#define RX_ANGLE_DAMPNING 20.0
#define RX_RATE_SENSITIVITY  3
#define ROLL_RMIN  -90
#define ROLL_RMAX  90
#define ROLL_WMIN  -90
#define ROLL_WMAX  90

#define PITCH_RMIN  -90
#define PITCH_RMAX  90
#define PITCH_WMIN  -90
#define PITCH_WMAX  90

#define YAW_RMIN  -180
#define YAW_RMAX  180
#define YAW_WMIN  -45
#define YAW_WMAX  45

//#define ACC_X_FILTER_OFFSET -2066
//#define ACC_Y_FILTER_OFFSET -1058
//#define ACC_Z_FILTER_OFFSET 15059

#define ACC_X_OFFSET  1836//1832
#define ACC_Y_OFFSET  910
#define ACC_Z_OFFSET  1059

#define GYRO_X_OFFSET  42
#define GYRO_Y_OFFSET  -30
#define GYRO_Z_OFFSET  16


#define SPLIT  0.99 //COMP-filter nr
#define RadToDeg 180.0/PI 

#define FAST_SAMPLE_TIME 1250 //400Hz using Micros() //1000 millisecond(1 sec) per 100 commands = 10 milliseconds per command
#define SLOW_SAMPLE_TIME 20 //50Hz using Millis()
#define PRINT_TIME 100
unsigned long printTimer = 0;
unsigned long slowLoopTimer = 0;
unsigned long fastLoopTimer = 0;
unsigned int timeChange = 0;

//thirty seconds
#define FAILSAFE_THRESHOLD   30000
bool failSafe = false;
/////////////////
//PID VARIABLES//
/////////////////

double ROLL_PID_KP = 50;
//double ROLL_PID_KI = 0.950;
//double ROLL_PID_KD = 0.011;
double ROLL_PID_KI = 0;
double ROLL_PID_KD = 0;
double ROLL_PID_MIN = -600.0;
double ROLL_PID_MAX = 600.0;

double PITCH_PID_KP = 50;
//double PITCH_PID_KI = 0.950;
//double PITCH_PID_KD = 0.011;
double PITCH_PID_KI = 0;
double PITCH_PID_KD = 0;
double PITCH_PID_MIN = -600.0;
double PITCH_PID_MAX = 600.0;

double YAW_PID_KP = 10;
double YAW_PID_KI =  0.0;
double YAW_PID_KD = 0.0;
//double YAW_PID_KP = 0.680;
//double YAW_PID_KI =  0.500;
//double YAW_PID_KD = 0.0001;
double YAW_PID_MIN = -600.0;
double YAW_PID_MAX = 600.0;

double ANGLEX_KP = 5.0;
double ANGLEX_KI = 0.02;
double ANGLEX_KD = -0.015;
double ANGLEX_MIN = -100.0;
double ANGLEX_MAX = 100.0;

double ANGLEY_KP = 5.0;
double ANGLEY_KI = 0.02;
double ANGLEY_KD = -0.015;
double ANGLEY_MIN = -100.0;
double ANGLEY_MAX = 100.0;

//Define the PID class
PIDCont PIDroll;
PIDCont PIDpitch;
PIDCont PIDyaw;
PIDCont PIDangleX;
PIDCont PIDangleY;

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

int accx_temp=0;
int accy_temp=0;
int accz_temp=0;

float angles[2]={
  0.0,0.0};
float gx_aver=0;
float gy_aver=0;
float gz_aver=0;
#define  ACC_HPF_NR  98  //high pass filter nr

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
#define filterSamples   9              // filterSamples should  be an odd number, no smaller than 3
int yawSmoothArray [filterSamples];   // array for holding raw sensor values for yaw 
int pitchSmoothArray [filterSamples];   // array for holding raw sensor values for yaw 
int rollSmoothArray [filterSamples];   // array for holding raw sensor values for yaw 
int yawsortedArray [filterSamples];
int pitchsortedArray [filterSamples];
int rollsortedArray [filterSamples];

#define gyroFilterSamples 17 // smooth will get rid of top and bottom 2 if size is 17
int gyroXSmoothArray [gyroFilterSamples];   // array for holding raw sensor values for yaw 
int gyroYSmoothArray [gyroFilterSamples];   // array for holding raw sensor values for yaw 
int gyroZSmoothArray [gyroFilterSamples];   // array for holding raw sensor values for yaw 
int gyroXsortedArray [gyroFilterSamples];
int gyroYsortedArray [gyroFilterSamples];
int gyroZsortedArray [gyroFilterSamples];

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

int receivedRoll;
int receivedPitch;
int receivedYaw;

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
#define MOTOR_RUN_LEVEL 600
//Value at which motors are not moving
#define MOTOR_ZERO_LEVEL 550

/////////////////
//LED VARIABLES//
/////////////////

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)



