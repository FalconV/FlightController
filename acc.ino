#include "common.h"



void AccelerometerInit() {
  uint8_t temp[1];
  uint8_t temp1;
  // Reset:
  hal.i2c->writeRegister(BMA180, RESET, 0xB6);
  //Wake up mode
  hal.i2c->writeRegister(BMA180, PWR, 0x10);
  //low pass filter,
  hal.i2c->readRegisters(BMA180, BW, 1, temp);
  temp1=temp[0]&0x0F;
  hal.i2c->writeRegister(BMA180, BW, temp1);
  // range +/- 2g
  hal.i2c->readRegisters(BMA180, RANGE, 1, temp);
  temp1=(temp[0]&0xF1) | 0x04;
  hal.i2c->writeRegister(BMA180, RANGE, temp1);
  
}

void AccelerometerRead(int * retval) {
 // read in the 3 axis data, each one is 14 bits
 int n = 6;
uint8_t result[5];

hal.i2c->readRegisters(BMA180, DATA, n, result);

retval[0] = ((result[0] | result[1] << 8) >> 2) + offx;

retval[1] = ((result[2] | result[3] << 8) >> 2) + offy;

retval[3] = ((result[4] | result[5] << 8) >> 2) + offz;
  
  
}
