#include "common.h"

void initGyro(){
  /*****************************************
 * ITG 3200
 * power management set to:
 * clock select = internal oscillator
 * no reset, no sleep mode
 * no standby mode
 * sample rate to = 125Hz
 * parameter to +/- 2000 degrees/sec
 * low pass filter = 5Hz
 * no interrupt
 ******************************************/
 
  hal.i2c->writeRegister(GYRO, G_PWR_MGM, 0x00);
  hal.i2c->writeRegister(GYRO, G_SMPLRT_DIV, 0x07);  // EB, 50, 80, 7F, DE, 23, 20, FF
  hal.i2c->writeRegister(GYRO, G_DLPF_FS, 0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
  hal.i2c->writeRegister(GYRO, G_INT_CFG, 0x00);
  
  int buf[3];
  getGyroscopeData (buf);
//  g_offx -= buf[0];
//  g_offy -= buf[1];
//  g_offz -= buf[2];
  xBuf[0]=xBuf[1]=xBuf[2]=xBuf[3]=0;
  yBuf[0]=yBuf[1]=yBuf[2]=yBuf[3]=0;
  zBuf[0]=zBuf[1]=zBuf[2]=zBuf[3]=0;
  
 
}

void getGyroscopeData (int * result) 
{
 /**************************************
 Gyro ITG-3200 I2C
 registers:
 temp MSB = 1B, temp LSB = 1C
 x axis MSB = 1D, x axis LSB = 1E
 y axis MSB = 1F, y axis LSB = 20
 z axis MSB = 21, z axis LSB = 22
 *************************************/
 int regAddress = 0x1B;
 uint8_t buff[G_TO_READ];
 // read the gyro data from the ITG3200
 uint8_t stat = hal.i2c->readRegisters(GYRO,regAddress,G_TO_READ, buff);
 
 result[0] = ((buff[2] << 8) | buff[3]) + g_offx;
 result[1] = ((buff[4] << 8) | buff[5]) + g_offy;
 result[2] = ((buff[6] << 8) | buff[7]) + g_offz;
 result[3] = (buff[0] << 8) | buff[1]; // temperature
// g_offx = 
}
 
