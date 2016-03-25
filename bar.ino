
#include "common.h"



char bmp085Read(unsigned char address) {
  uint8_t data[1];
  hal.i2c->readRegisters(BMP085_ADDRESS, address, 1, data);
  return *data;
}

int bmp085ReadInt(unsigned char address) {
  uint8_t temp[2];
  hal.i2c->readRegisters(BMP085_ADDRESS, address, 2, temp);
  return  (((int16_t)temp[0]<<8)|temp[1]);
}

//read the uncompensated temperature value
unsigned int bmp085ReadUT(){
  unsigned int ut;
  //Write 0x2E int register 0xF4 to request temperature reading
  hal.i2c->writeRegister(BMP085_ADDRESS, 0xF4, 0x2E);
 //wait for conversion (refer to OSS definition)
  hal.scheduler->delay(5);
 //read two bytes from registers 0xF6 and 0xF7 to get an int
 ut = bmp085ReadInt(0xF6);
  return ut; 
  
}

//read the uncompensated pressure value
unsigned long bmp085ReadUP(){
  uint8_t data[3];
  unsigned long up = 0;
// Write 0x34+(OSS<<6) into register 0xF4
// Request a pressure reading w/ oversampling setting
hal.i2c->writeRegister(BMP085_ADDRESS, 0xF4, 0x34 + (OSS<<6));
// Wait for conversion, delay time dependent on OSS
hal.scheduler->delay(2 + (3<<OSS));

hal.i2c->readRegisters(BMP085_ADDRESS, 0xF6, 3, data);

up = ((unsigned long)data[0]<<16)|((unsigned long)data[1]<<8)|((unsigned long)data[2])>>(8-OSS);
//hal.console->printf_P(PSTR("UP = %d\t"), up);
return up;  
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = ((((long)ac1)*4 + x3)<<OSS + 2)>>2;
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = ac4 * (unsigned long)(x3 + 32768)>>15;
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  //hal.console->printf_P(PSTR("b7=0x%lx\t"),b7);
  if (b7 < 0x80000000){
    p = (b7<<1)/b4;
  }
  else{
    p = (b7/b4)<<1;
  }
  //hal.console->printf_P(PSTR("P=%l\t"),p);
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  //hal.console->printf_P(PSTR("P=%li \t"),p);
  return p; 
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut)
{
long x1, x2;
x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
x2 = ((long)mc << 11)/(x1 + md);
b5 = x1 + x2;
return ((b5 + 8)>>4);  
}

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
ac1 = bmp085ReadInt(0xAA);
ac2 = bmp085ReadInt(0xAC);
ac3 = bmp085ReadInt(0xAE);
ac4 = bmp085ReadInt(0xB0);
ac5 = bmp085ReadInt(0xB2);
ac6 = bmp085ReadInt(0xB4);
b1 = bmp085ReadInt(0xB6);
b2 = bmp085ReadInt(0xB8);
mb = bmp085ReadInt(0xBA);
mc = bmp085ReadInt(0xBC);
md = bmp085ReadInt(0xBE);
//hal.console->printf_P(PSTR("ac1=%d\nac2=%d\nac3=%d\nac4=%d\nac5=%d\nac6=%d\nb1=%d\nb2=%d\nmb=%d\nmc=%d\nmd=%d\n"),
     // ac1, ac2, ac3, ac4, ac5, ac6, b1, b2, mb, mc, md);
}

//this function is used just to make sure that the temperature is always measured before the pressure
void bmp085GetTempPress (){
temperature = bmp085GetTemperature(bmp085ReadUT());
pressure = bmp085GetPressure(bmp085ReadUP());
//hal.console->printf_P(PSTR("pressure=%li\t"), pressure);
}

//calculate height based on standard temperature/pressure at sea level
float bmp085GetAltitude(){
  /* altitude calculated using:
   *  h = h_b + (T_b/L_b)*[(P/P_b)^((-R*L_b)/(g_0*M)) - 1]
   *  Where:
   *  P_b = pressure at sea level [Pa]
   *  T_b = temperature at sea level [K]
   *  L_b = standard temperature lapse rate [K/m] = -0.0065 [K/m]
   *  h = height about sea level [m]
   *  h_b = height at the bottom of the atmospheric layer [m]
   *  R = universal gas constant = 8.31432 [(N*m)/(mol*K)]
   *  g_0 = gravitation acceleration constant = 9.80665 [m/s^2]
   *  M = molar mass of Earth's air = 0.02889644 [kg/mol]
   */
  double P_b = 101325; // [Pa]
  double T_b = 26 + 273.15 ; // [K]
  double L_b = -0.0065; // [K/m]
  double h_b = 0; //using sea level [m]
  double R = 8.31432; // [(N*m0/(mol*K)]
  double g_0 = 9.80655; //[m/s^2]
  double M = 0.0289644; //[kg/mol]
  double h; // [m]
  
  bmp085GetTempPress ();
  double P = pressure; // [Pa]
  //double sqrbracket = pow((P/P_b), (R * L_b * (-1))/(g_0 * M)) - 1;
  //Serial.print(sqrbracket);
  h = h_b + (T_b/L_b)*(pow((P/P_b),((-R*L_b)/(g_0*M))) - 1);
  //Serial.print(h);
  //return h;
  
  h = 44330 * (1 - pow(((P/100)/1013.25), (1/5.255)));
  //hal.console->printf_P(PSTR("H=%f\t"), h);
  return h;
}

void bmp085Init(){
  bmp085Calibration();
//  initialAltitude = bmp085GetAltitude() * 3.28084; // convert to feet
//  avgAlt = initialAltitude;
}
