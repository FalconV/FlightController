

#include "common.h"
#include <AP_Common.h>
#include <AP_ADC.h>
//#include <AP_InertialSensor.h>
#include <math.h>
#include <AP_Progmem.h>
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
#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

#if CONFIG_HAL_BOARD == HAL_BOARD_APM1
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;


//AP_HAL::Semaphore *sem;

void setup() {
  
    // PID Configuration
    pids[PID_PITCH_RATE].kP(0.7);
//    pids[PID_PITCH_RATE].kI(1);
//    pids[PID_PITCH_RATE].imax(50);
    pids[PID_PITCH_RATE].kI(0);
    pids[PID_PITCH_RATE].imax(0);
    
//    pids[PID_ROLL_RATE].kP(0.82);
    pids[PID_ROLL_RATE].kP(0.855);
//    pids[PID_ROLL_RATE].kI(1);
//    pids[PID_ROLL_RATE].imax(25);
    pids[PID_ROLL_RATE].kI(0.2);
    pids[PID_ROLL_RATE].imax(5);
  
    pids[PID_YAW_RATE].kP(2.7);
//    pids[PID_YAW_RATE].kI(1);
//    pids[PID_YAW_RATE].imax(50);
    pids[PID_YAW_RATE].kI(0);
    pids[PID_YAW_RATE].imax(0);
  
    pids[PID_PITCH_STAB].kP(4.5);
    pids[PID_ROLL_STAB].kP(4.5);
    pids[PID_YAW_STAB].kP(10);
    
    
    sensorsInit();
    
    hal.rcout->set_freq(0xF, 490);
    hal.rcout->enable_mask(0xFF);
    
    sensor_timer = hal.scheduler->micros();
    baro_timer = hal.scheduler->micros();
    command_timer = hal.scheduler->micros();
    lastPkt = hal.scheduler->millis();
    pidTimer = hal.scheduler->micros();
    
    hal.console->println("ready");
    
    
}

void loop() {
  
    static float yaw_target = 0;
  
    static uint32_t last_print;
    bytesAvail = hal.console->available();
//    uint32_t t = hal.scheduler->micros();
//    hal.console->printf_P(PSTR("trans time=%d us\n"),hal.scheduler->micros() - t);

    if (bytesAvail > 0) { //communications lasts ~2750 us or 2.75 ms w/ echo, ~340 us w/o
      processMsg(bytesAvail);
    }
    else {
        // accumulate values at 100Hz (barometer cannot go any faster than 100Hz)
      if ((hal.scheduler->micros()- sensor_timer) > 10000) { // sensor readings take ~820 us
          updateSensorVals();
         
      }
      if ((hal.scheduler->micros() - pidTimer) > 10000){ //update PIDs at 100Hz. Whole function takes ~1150 us, or 1.15ms
      
        //mapping takes ~150 us
        rcthr = channels[2];
        rcyaw = map(channels[3], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
        rcpit = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, -45, 45);
        rcroll = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, -45, 45);
//        
        if(rcthr > RC_THR_MIN + 100){  
//          hal.console->println("in pid calc");
          //stab pids take ~350us
//          bytesAvail = hal.console->available();
//          if (bytesAvail >0)
//            processMsg(bytesAvail);
//          float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch,1), -250, 250);
          bytesAvail = hal.console->available();
          if (bytesAvail >0)
            processMsg(bytesAvail);
          float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll,1), -250, 250);
          bytesAvail = hal.console->available();
          if (bytesAvail >0)
            processMsg(bytesAvail);
//          float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);
//          bytesAvail = hal.console->available();
//          if (bytesAvail >0)
//            processMsg(bytesAvail);
          
          float pitch_stab_output=0;
//          float roll_stab_output=0;
          float yaw_stab_output=0;
          
          if (abs(rcyaw) > 5){
            yaw_stab_output = rcyaw;
            yaw_target = yaw;
          }
          //rate pids take ~650 us:
//          bytesAvail = hal.console->available();
//          if (bytesAvail >0)
//            processMsg(bytesAvail);
//          long pitch_output = (long) constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - (gyrX / GYR_P_SCALE),1),-500,500);
          bytesAvail = hal.console->available();
          if (bytesAvail >0)
            processMsg(bytesAvail);
          long roll_output = (long) constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - (gyrY / GYR_R_SCALE),1),-500,500);
          bytesAvail = hal.console->available();
          if (bytesAvail >0)
            processMsg(bytesAvail);
//          long yaw_output = (long) constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - (gyrZ / GYR_Y_SCALE),1),-500, 500);
//          bytesAvail = hal.console->available();
//          if (bytesAvail >0)
//            processMsg(bytesAvail);
          
//          pitch_output /= 2;
//          roll_output/=2;
//          yaw_output/=2;
          long pitch_output = 0;
//          long roll_output = 0;
          long yaw_output = 0;
          
//          hal.console->printf_P(PSTR("pitch_stab_output = %f\t"),pitch_stab_output);
//          hal.console->printf_P(PSTR("roll_stab_output = %f\t"),roll_stab_output);
//          hal.console->printf_P(PSTR("yaw_stab_output = %f\t"),yaw_stab_output);
//          hal.console->printf_P(PSTR("pitch_output = %li\t"),pitch_output);
//          hal.console->printf_P(PSTR("roll_output = %li\t"),roll_output);
//          hal.console->printf_P(PSTR("yaw_output = %li\n"),yaw_output);
          //writing to motors takes ~20 us
          hal.rcout->write(MOTOR_FL, rcthr + roll_output + pitch_output - yaw_output);
          hal.rcout->write(MOTOR_BL, rcthr + roll_output - pitch_output + yaw_output);
          hal.rcout->write(MOTOR_FR, rcthr - roll_output + pitch_output + yaw_output);
          hal.rcout->write(MOTOR_BR, rcthr - roll_output - pitch_output - yaw_output);
//          hal.console->printf_P(PSTR("FL = %li\t"),rcthr + roll_output + pitch_output - yaw_output);
//          hal.console->printf_P(PSTR("BL = %li\t"),rcthr + roll_output - pitch_output + yaw_output);
//          hal.console->printf_P(PSTR("FR = %li\t"), rcthr - roll_output + pitch_output + yaw_output);
//          hal.console->printf_P(PSTR("BR = %li\n"), rcthr - roll_output - pitch_output - yaw_output);
          
        } else { // motors off
          hal.rcout->write(MOTOR_FL, 1000);
          hal.rcout->write(MOTOR_BL, 1000);
          hal.rcout->write(MOTOR_FR, 1000);
          hal.rcout->write(MOTOR_BR, 1000);
          
          yaw_target = yaw;
          
          //reset pids while on the ground
          
          for(int i = 0; i < 6; i++)
            pids[i].reset_I();
            
            
          bytesAvail = hal.console->available();
            
        }
        
      }
      if ((hal.scheduler->millis() - lastPkt) > 100){
        hal.console->println("TIMEOUT");
        lastPkt = hal.scheduler->millis();
        timeouts++;
      }
    }
    
    
}

uint8_t verify_chksum(char *str, char *chk){
   uint32_t nc = 0;
//   hal.console->printf_P(PSTR("received:%s*%s*\n"),str,chk);
   for (int i =0; i < strlen(str); i++){
     nc += str[i];
   }
//   hal.console->printf_P(PSTR("nc = %li\n"),nc);
   
   long chkL = strtol(chk, NULL, 10);
   //hal.console->printf_P(PSTR("chkL = %li\n"), chkL);
   if (chkL == (long)nc){
     return true;
   }
   
   return false;
 }


void getInitialReadings(){
    static uint32_t last_print;
  while (hal.scheduler->millis() <= 1000){
    if ((hal.scheduler->micros()- sensor_timer) > 20000L) {
          updateSensorVals();
          sensor_timer = hal.scheduler->micros();
      }
    if ((hal.scheduler->millis()- last_print) >= 50) {
        last_print = hal.scheduler->millis();
      printSensors();
    }
  }
}

long map(long x, long in_min, long in_max, long out_min, long out_max){
 return (x - in_min)*(out_max-out_min)/(in_max-in_min)+out_min; 
}


/*#elif CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;*/
#else // Non-APM1
#warning AP_Baro_BMP085_test built as stub for APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
void setup() {}
void loop() {}
#endif

AP_HAL_MAIN();
