#ifndef COMMON_H
#define COMMON_H
#include <AP_Common.h>
#include <AP_ADC.h>
//#include <AP_InertialSensor.h>
#include <math.h>
//#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_Buffer.h>
#include <Filter.h>
#include <AP_Baro.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <GCS_MAVLink.h> 
#include <PID.h>
#include <Arduino.h>


#define HMC5883L_address  0x1E

// **** Gyro defines **** //
//gyro is connected to GND in our case:
#define GYRO 0x68 //  when AD0 is connected to GND ,gyro address is 0x68.
//#define GYRO 0x69   when AD0 is connected to VCC ,gyro address is 0x69 
#define G_SMPLRT_DIV 0x15
#define G_DLPF_FS 0x16
#define G_INT_CFG 0x17
#define G_PWR_MGM 0x3E
#define G_TO_READ 8 // 2 bytes for each axis x, y, z 
#define GYR_BUF_SIZE 4
#define GYR_OFFBUF_SIZE  100;
#define GYR_P_SCALE  18.4486
#define GYR_R_SCALE  18.5333
#define GYR_Y_SCALE  18.4611
#define GYR_WEIGHT  0.99
// offsets are chip specific. 
int g_offx = 55;
int g_offy = 8;
int g_offz = -3;
float hx, hy, hz, turetemp;
float gyrX = 0, gyrY = 0, gyrZ = 0;
float Heading;
float m_Scale; //mag scale
float initHeading;
float xBuf[GYR_BUF_SIZE];
float yBuf[GYR_BUF_SIZE];
float zBuf[GYR_BUF_SIZE];
//int xBuf[GYR_BUF_SIZE] = {0,0,0,0}, yBuf[GYR_BUF_SIZE] = {0,0,0,0}, zBuf[GYR_BUF_SIZE] = {0,0,0,0};
int gyrBufIter = 0, gyrBufIter_1 = 0, gyrBufIter_2 = 0, gyrBufIter_3 = 0;

float pitch = 0.0;
float roll = 0.0;
float yaw = 0.0;
float dt=0.02;

float kalRollAngle, kalPitchAngle, kalYawAngle;
//Kalman kalRoll, kalPitch;
uint32_t kalRollTimer;
uint32_t kalPitchTimer;
uint32_t kalYawTimer;
float kalDeltaT;


// **** Accelerometer defines **** //
#define BMA180 0x40  //address of the accelerometer
#define RESET 0x10   
#define PWR 0x0D
#define BW 0X20
#define RANGE 0X35
#define DATA 0x02
// **** acceleromter variables **** //
int offx = 31;  
int offy = 47;   
int offz = -23; 
//int acc[3];
float scaledAcc[4];

//char degSymb = (char)176;


// **** Barometer defines **** //
#define BMP085_ADDRESS 0x77  // I2C address of BMP085
// **** barometer variables **** //
// Oversampling Setting: 
//0-Ultra Low Power, 1-Standard, 2-High Res, 3-Ultra High Res
/* OSS-Description | Conversion Time [ms]
0-Ultra Low Power | 4.5
1-Standard | 7.5
2-High Res | 13.5
3-Ultra High Res | 25.5
*/
const unsigned char OSS = 0;  
// Calibration values
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;
// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 
short temperature;
long pressure;
//unsigned int initialAltitude;
float avgAlt;
AP_Baro_BMP085 bmp085;
uint32_t sensor_timer;
uint32_t baro_timer;


uint32_t gyro_timer;
uint32_t acc_timer;
uint32_t mag_timer;


uint8_t sensor_state = 0;




/*******Motor variables********/
#define MOTOR_FR  4
#define MOTOR_BL  5
#define MOTOR_FL  7
#define MOTOR_BR  6
//long rcthr, roll_output, pitch_output, yaw_output;

/*******************PID vars****************/

#define PID_PITCH_RATE  0
#define PID_ROLL_RATE  1
#define PID_PITCH_STAB  2
#define PID_ROLL_STAB  3
#define PID_YAW_RATE  4
#define PID_YAW_STAB  5

PID pids[6]; 
uint32_t pidTimer;

#define RC_THR_MIN 1199
#define RC_YAW_MIN 1199
#define RC_YAW_MAX 1900
#define RC_PIT_MIN 1199
#define RC_PIT_MAX 1900
#define RC_ROL_MIN 1199
#define RC_ROL_MAX 1900

long rcthr, rcyaw, rcpit, rcroll;


/****** comms vars*******/
uint32_t command_timer;
int console_value;
char cmd_buf[255];
int cmd_buf_offset = 0;
static int16_t channels[4] = {0,0,0,0};
char *chksum = "0";
int misses = 0;
int packets = 0;
int timeouts = 0;
float txrx_health = 0.0;
uint32_t lastPkt;
int bytesAvail;

#endif
