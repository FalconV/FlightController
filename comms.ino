

void processMsg(int bytesAvail){
  while (bytesAvail > 0){
    char c = (char)hal.console->read();
    
    if (c == '\n'){
      cmd_buf[cmd_buf_offset] = '\0';
      //process cmd
      //expected format: roll,pitch,throttle,yaw*chksum*
      char *str = strtok(cmd_buf, "*"); // str = roll,pitch,throtle,yaw
      char *chk = strtok(NULL, "*"); // chk = chksum
      
      if (verify_chksum(str,chk)) {
        char *ch = strtok(str, ","); //first channel
        channels[0] = (uint16_t)strtol(ch, NULL, 10); // parse
//        hal.console->printf_P(PSTR("ch[0]=\t%d\t"),channels[0]);
        for (int i =1; i <4; i++) { //loop through the final 3 channels
          char *ch = strtok(NULL, ",");
          channels[i] = (uint16_t)strtol(ch, NULL, 10);
//          hal.console->printf_P(PSTR("ch[%d]=\t%d"),i,channels[i]);
        }  
//        hal.console->println();
        lastPkt = hal.scheduler->millis(); // update last valid packet
        hal.console->println("OK");
      }    
      else {
//        misses++;
        lastPkt = hal.scheduler->millis();
        hal.console->println("MISS");
      }
      cmd_buf_offset = 0;      
//      packets++;
//      txrx_health = 1.0 - ((float)misses/(float)packets);
      
    }
    else if (c != '\r') {
      cmd_buf[cmd_buf_offset++] = c;
    }
    bytesAvail = hal.console->available();
  }
}
