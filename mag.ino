


float setScale(float gauss)
{
  uint8_t regValue = 0x00;
  if(gauss == 0.88)
  {
    regValue = 0x00;
    m_Scale = 0.73;
  }
  else if(gauss == 1.3)
  {
    regValue = 0x01;
    m_Scale = 0.92;
  }
  else if(gauss == 1.9)
  {
    regValue = 0x02;
    m_Scale = 1.22;
  }
  else if(gauss == 2.5)
  {
    regValue = 0x03;
    m_Scale = 1.52;
  }
  else if(gauss == 4.0)
  {
    regValue = 0x04;
    m_Scale = 2.27;
  }
  else if(gauss == 4.7)
  {
    regValue = 0x05;
    m_Scale = 2.56;
  }
  else if(gauss == 5.6)
  {
    regValue = 0x06;
    m_Scale = 3.03;
  }
  else if(gauss == 8.1)
  {
    regValue = 0x07;
    m_Scale = 4.35;
  }
  else
    return 0.0;
  
  // Setting is in the top 3 bits of the register.
  regValue = regValue << 5;
  hal.i2c->writeRegister(0x1E, 0x01, regValue);
  
  return m_Scale;
}

char* compassDir (float HeadingDeg) {
  if (HeadingDeg <= 22.5 || HeadingDeg > 337.5) {
    return "N";
  }
  else if (HeadingDeg <= 67.5) {
    return "NE";
  }
  else if (HeadingDeg <= 112.5) {
    return "E";
  }
  else if (HeadingDeg <= 157.5) {
    return "SE";
  }
  else if (HeadingDeg <= 202.5) {
    return "S";
  }
  else if (HeadingDeg <= 247.5) {
    return "SW";
  }
  else if (HeadingDeg <= 292.5) {
    return "W";
  }
  else if (HeadingDeg <=  337.5) {
    return "NW";
  }
  else {
    return "ERROR";
  }
}

float getHeadingDeg(){
  uint8_t data[6];
    //read 6 bytes (x,y,z) from the device
    uint8_t stat = hal.i2c->readRegisters(HMC5883L_address,0x03,6, data);
    
    if (stat == 0){
      int x, y, z;
      //scaled axes
      //x and y axis offsets calculated by 
      x = ((data[0] << 8) | data[1])*m_Scale - 15.04587;
      z = ((data[2] << 8) | data[3])*m_Scale;
      y = ((data[4] << 8) | data[5])*m_Scale + 209.38376;
      
      //hal.console->printf_P(PSTR("x: %d y: %d z: %d\t"), x, y, z);
       
      float Heading = atan2(y, x);
      float declinationAngle = -0.0471;
      Heading += declinationAngle;
      //readings were off by exactly 180 degrees, this corrects that
      Heading += PI;
      if(Heading < 0){
        Heading += 2*PI;
      }
      if (Heading > 2*PI){
        Heading -= 2*PI;
      }
      float headingDegrees = Heading * 180/M_PI;
      char* Direction = compassDir(headingDegrees);
      //if (printEn)
        //hal.console->printf_P(PSTR("%f\t%s\t"), headingDegrees, Direction); 
      return headingDegrees;
    } else { 
        hal.console->printf_P(PSTR("i2c error: status %d\t"), (int)stat);
        return 0.0;
    }
    

}
  
int magInit(){
    hal.console->printf_P(PSTR("\nInitializing HMC5883L at address %x\r\n"),
                                HMC5883L_address);
    //set to continuous measurement mode
    uint8_t stat = hal.i2c->writeRegister(HMC5883L_address,0x02,0x00);// address, register num, value
    if (stat == 0) {
        hal.console->printf_P(PSTR("successful init\r\n"));
    } else {
        hal.console->printf_P(PSTR("failed init: return status %d\r\n"),
                (int)stat);
        for(;;);
    }
    
    
    //error = compass.SetScale(1.3); // Set the scale to +/- 1.3 Gauss
    m_Scale = setScale(1.3);
    if (m_Scale == 0){
      hal.console->printf_P(PSTR("failed to set scale\r\n"));
      for(;;);
    }
    
    //get initial direction
    initHeading = getHeadingDeg();
}  
  
