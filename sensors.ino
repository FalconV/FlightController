#include "common.h"
float filterAngle;
Kalman kalRoll, kalPitch, kalYaw;

void updateGyro(){
  int gyro[4];
  getGyroscopeData(gyro); //now gyro has data
  hx = gyro[0] / 14.375;
  hy = gyro[1] / 14.375;
  hz = gyro[2] / 14.375;
  
  gyrBufIter = (gyrBufIter + 1) % 4;
  //for some reason, mod doesn't work with subtractions, so...
  gyrBufIter_1 = (gyrBufIter + 3) % 4; // equiv to -1
  gyrBufIter_2 = (gyrBufIter + 2) % 4; // equiv to -2
  gyrBufIter_3 = (gyrBufIter + 1) % 4; // equiv to -3
  
  xBuf[gyrBufIter] = hx;
  yBuf[gyrBufIter] = hy;
  zBuf[gyrBufIter] = hz;
  
//  perform runge-kutta integration
  gyrY = gyrY + (yBuf[gyrBufIter_3] + 2*yBuf[gyrBufIter_2] + 2*yBuf[gyrBufIter_1] + yBuf[gyrBufIter])/6.0;
  gyrZ = gyrZ + (zBuf[gyrBufIter_3] + 2*zBuf[gyrBufIter_2] + 2*zBuf[gyrBufIter_1] + zBuf[gyrBufIter])/6.0;
  gyrX = gyrX + (xBuf[gyrBufIter_3] + 2*xBuf[gyrBufIter_2] + 2*xBuf[gyrBufIter_1] + xBuf[gyrBufIter])/6.0;
  
  
  //temperature = 35 + ((double) (gyro[3] + 13200)) / 280; //temperature of gyro
  
  //hal.console->printf_P(PSTR("Gyro:\tx=%d\ty=%d\tz=%d\t"),hx, hy, hz);

}

void updateMag() {
  Heading = getHeadingDeg(); //store heading in global variable
  //hal.console->printf_P(PSTR("\tHead:%f\t"), Heading);
  
}

void updateAcc() {
  int acc[3];
  int n = 6;
  //AccelerometerRead(acc);
  uint8_t result[6];
  hal.i2c->readRegisters(BMA180, DATA, n, result);
  
  acc[0] = ((result[0] | result[1] << 8) >> 2) + offx;
  acc[1] = ((result[2] | result[3] << 8) >> 2) + offy;
  acc[2] = ((result[4] | result[5] << 8) >> 2) + offz;
  
  scaledAcc[0] = acc[0]/4096.0;
  scaledAcc[1] = acc[1]/4096.0;
  scaledAcc[2] = acc[2]/4096.0;
  
  //hal.console->printf_P(PSTR("ACC:\tx=%f\ty=%f\tz=%f\t"),scaledAcc[0], scaledAcc[1], scaledAcc[2]);
  
}

void updateAlt() {
    float tmp_float;
    bmp085.accumulate();
    tmp_float = ( bmp085.get_pressure() / 101325.0);
    tmp_float = pow(tmp_float, 0.190295);
    float alt = 44330.0 * (1.0 - tmp_float);
    alt *= 3.28084; // convert to feet
    avgAlt -= avgAlt / 5;
    avgAlt += alt / 5;
}

float getPitch(){
//  kalDeltaT = (float)(hal.scheduler->micros() - kalPitchTimer)/100000.0;
//  kalPitchTimer = hal.scheduler->micros();
//  int x = 0, y = 1, z = 2;
//  float accPitch = atan2(scaledAcc[y],sqrt(scaledAcc[x]*scaledAcc[x] + scaledAcc[z]*scaledAcc[z]))*180/PI;
//  if (abs(kalRollAngle) > 90){
//    hy = -hy;
//  }
//  kalPitchAngle = kalPitch.getAngle(accPitch, hy, kalDeltaT);
  return kalPitchAngle;
}

float getRoll() {
  
//  kalDeltaT = (float)(hal.scheduler->micros() - kalRollTimer)/100000.0;
//  kalRollTimer = hal.scheduler->micros();
//  int x = 0, y = 1, z = 2;
//  float accRoll = atan2((scaledAcc[x]),scaledAcc[z])*180/PI;
//  if ((accRoll < -90 && kalRollAngle > 90) || (accRoll > 90 && kalRollAngle < -90)){
//    kalRoll.setAngle(accRoll);
//    kalRollAngle = accRoll;
//  } else {
//    kalRollAngle = kalRoll.getAngle(accRoll, hx, kalDeltaT);
//  }
  
  return kalRollAngle;
}

float getYaw() {
//  kalDeltaT = (float)(hal.scheduler->micros() - kalYawTimer)/100000.0;
//  kalYawTimer = hal.scheduler->micros();
//  int x = 0, y = 1, z = 2;
////  float accYaw = atan2(scaledAcc[z],sqrt(scaledAcc[x]*scaledAcc[x] + scaledAcc[z]*scaledAcc[z]))*180/PI;
//  float magYaw = initHeading - getHeadingDeg();
//  kalYawAngle = kalYaw.getAngle(magYaw, hz, kalDeltaT);
////  hal.console->printf_P(PSTR("magYaw=\t%.2f\theading=\t%.2f"),magYaw, getHeadingDeg());
  return kalYawAngle;

//  float gyrYaw = gyrY / GYR_Y_SCALE;
//  return GYR_WEIGHT * gyrYaw + (1 - GYR_WEIGHT) * accYaw;
//  hal.console->printf_P(PSTR("gyrY-magY=%.2f\t"),gyrYaw-magYaw);
//  return GYR_WEIGHT * gyrYaw + (1 - GYR_WEIGHT) * magYaw;
//  filterAngle = yaw;
//  yaw = complimentary_filter(magYaw, hz);  
//  return yaw;
}


void updateSensorVals(){
  switch (sensor_state) {
          case 0: {
          //read from Gyro:
              updateGyro();
//              updateAcc(); 
//              updateMag(); 
//              float headingDegrees = getHeadingDeg();
//              char* Direction = compassDir(headingDegrees);
//              if ((hal.scheduler->micros()- baro_timer) > 20000L) {
//                  updateAlt();
//                  baro_timer = hal.scheduler->micros();
//              }
//              sensor_state = 0;
              sensor_state++;
//              hal.console->printf_P(PSTR(" Raw Gyro: x=%f\ty=%f\tz=%f\t|"),hx, hy, hz);
//              hal.console->printf_P(PSTR(" Integ Gyro: gx=%f\tgy=%f\tgz=%f\t|"),gyrX, gyrY, gyrZ);
              bytesAvail = hal.console->available();
              break;
            }
            case 1: {
              //read from accelerometer:
              updateAcc(); 
              sensor_state++;
//              hal.console->printf_P(PSTR(" ACC: x=%.2f\ty=%.2f\tz=%.2f\t|"),scaledAcc[0], scaledAcc[1], scaledAcc[2]);
//              hal.console->printf_P(PSTR("pit = %.2f\trol = %.2f\tyaw = %.2f\t|"),getPitch(),getRoll(), getYaw());
              bytesAvail = hal.console->available();
              break;
            }
            case 2: {
              //read from magnetometer:
              updateMag(); 
              sensor_state++;    
              float headingDegrees = getHeadingDeg();
              char* Direction = compassDir(headingDegrees);
//              hal.console->printf_P(PSTR(" HEAD: %.2f%c\t%s\t|"), headingDegrees, degSymb, Direction); 
              bytesAvail = hal.console->available();
              break;
            }
            case 3: {
              //read temperature/pressure, then get altitude
              // accumulate values at 100Hz
              if ((hal.scheduler->micros()- baro_timer) > 20000L) {
                  updateAlt();
                  baro_timer = hal.scheduler->micros();
              }
              sensor_state = 0;
//              hal.console->printf_P(PSTR("%d feet abv sea\t%d feet abv start |\t"), (int)avgAlt, (unsigned int)(avgAlt - initialAltitude));
//              hal.console->printf_P(PSTR("%d ft abv sea\t|"), (int)avgAlt);
              bytesAvail = hal.console->available();
              break;
            }
            
              pitch = getPitch();
              roll = getRoll();
              yaw = getYaw();
            
      }
      //calculate delta time for kalman filter
      kalDeltaT = (float)(hal.scheduler->micros() - kalRollTimer)/100000.0;
      kalRollTimer = hal.scheduler->micros();
      
//      hal.console->printf_P(PSTR("pitch = %.2f\troll = %.2f\tyaw = %.2f"), pitch, roll, yaw);
//      hal.console->println();
      sensor_timer = hal.scheduler->micros();
      bytesAvail = hal.console->available();
       
}

void sensorsInit(){  
  int x = 0, y = 1, z = 2;
  magInit();
  initGyro();
  AccelerometerInit();
  //bmp085Init();
  bmp085.init();
//  hal.console->printf_P(PSTR("initial scaledAccs: x=%.2f\ty=%.2f\tz=%.2f\n"));
//  hal.console->printf_P(PSTR("initial angle setting: Roll=%.2f\tPitch=%.2f\n"), atan2((scaledAcc[x]),scaledAcc[z])*180/PI,atan2(scaledAcc[y],sqrt(scaledAcc[x]*scaledAcc[x] + scaledAcc[z]*scaledAcc[z]))*180/PI);
  kalRoll.setAngle(atan2((scaledAcc[x]),scaledAcc[z])*180/PI);
  kalPitch.setAngle(atan2(scaledAcc[y],sqrt(scaledAcc[x]*scaledAcc[x] + scaledAcc[z]*scaledAcc[z]))*180/PI);
  kalYaw.setAngle(initHeading - getHeadingDeg());
  kalRollTimer = kalPitchTimer = kalYawTimer = hal.scheduler->micros();
}

  
  
void printSensors() {
    float tmp_float;
    uint32_t start = hal.scheduler->micros();
    bmp085.read();
    uint32_t read_time = hal.scheduler->micros() - start;
    if (! bmp085.healthy) {
        hal.console->println("not healthy");
        return;
    }
    
//    hal.console->printf_P(PSTR("%d feet abv sea\t%d feet abv start |\t"), (int)avgAlt, (unsigned int)(avgAlt - initialAltitude));
    hal.console->printf_P(PSTR("%d ft abv sea\t|"), (int)avgAlt);
    hal.console->printf_P(PSTR(" ACC: x=%.2f\ty=%.2f\tz=%.2f\t|"),scaledAcc[0], scaledAcc[1], scaledAcc[2]);
    hal.console->printf_P(PSTR("pit = %.2f\trol = %.2f\t|"),getPitch(),getRoll);
    hal.console->printf_P(PSTR(" Gyro: x=%d\ty=%d\tz=%d\t|"),hx, hy, hz);
    
    
    float headingDegrees = getHeadingDeg();
    char* Direction = compassDir(headingDegrees);
//    hal.console->printf_P(PSTR(" HEAD: %.2f%c\t%s\t|"), headingDegrees, degSymb, Direction); 
    hal.console->println();
    bytesAvail = hal.console->available();
}
