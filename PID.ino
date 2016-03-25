
void runPIDs() {
//  
//        //mapping takes ~150 us
//        rcthr = channels[2];
//        rcyaw = map(channels[3], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
//        rcpit = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, -45, 45);
//        rcroll = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, -45, 45);
//        
////        
//        if(rcthr > RC_THR_MIN + 100){  
//          //stab pids take ~350us
//          float pitch_stab_output = constrain(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch,1), -250, 250);
//          float roll_stab_output = constrain(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll,1), -250, 250);
//          float yaw_stab_output = constrain(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);
//          
//          float pitch_stab_output;
//          float roll_stab_output;
//          float yaw_stab_output;
//          
//          if (abs(rcyaw) > 5){
//            yaw_stab_output = rcyaw;
//            yaw_target = yaw;
//          }
//          //rate pids take ~650 us:
////          long pitch_output = (long) constrain(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - (gyrX / GYR_P_SCALE),1),500,500);
////          long roll_output = (long) constrain(pids[PID_ROLL_RATE].get_pid(roll_stab_output - (gyrY / GYR_R_SCALE),1),-500,500);
////          long yaw_output = (long) constrain(pids[PID_YAW_RATE].get_pid(yaw_stab_output - (gyrZ / GYR_Y_SCALE),1),-500, 500);
//
//          long pitch_output;
//          long roll_output;
//          long yaw_output;
//          
//          //writing to motors takes ~20 us
//          hal.rcout->write(MOTOR_FL, rcthr + roll_output + pitch_output - yaw_output);
//          hal.rcout->write(MOTOR_BL, rcthr + roll_output - pitch_output + yaw_output);
//          hal.rcout->write(MOTOR_FR, rcthr - roll_output + pitch_output + yaw_output);
//          hal.rcout->write(MOTOR_BR, rcthr - roll_output - pitch_output - yaw_output);
//          
//        } else { // motors off
//          hal.rcout->write(MOTOR_FL, 1000);
//          hal.rcout->write(MOTOR_BL, 1000);
//          hal.rcout->write(MOTOR_FR, 1000);
//          hal.rcout->write(MOTOR_BR, 1000);
//          
//          yaw_target = yaw;
//          
//          //reset pids while on the ground
//          
//          for(int i = 0; i < 6; i++)
//            pids[i].reset_I();
//            
//            
//          bytesAvail = hal.console->available();
//            
//        }
        
}


