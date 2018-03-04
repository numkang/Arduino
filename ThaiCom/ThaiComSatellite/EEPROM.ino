//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                               EEPROM function                            //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#include <EEPROM.h>

void initEEPROM(){
  uint8_t check1 = EEPROM.read(0);
  if(check1 != 1){  
    /////// DEFAULT ANGGLE ///////
    initAzim = 0;
    initElev = 45;
    
    poten_azim_min = 140;
    poten_azim_max = 839;
    poten_elev_min = 177;
    poten_elev_max = 838;
    
    EEPROM.write(2,initAzim);
    delay(4);
    EEPROM.write(3,initElev);
    delay(4);
    EEPROM.write(4,poten_azim_min);
    delay(4);
    EEPROM.write(5,poten_azim_max);
    delay(4);
    EEPROM.write(6,poten_elev_min);
    delay(4);
    EEPROM.write(7,poten_elev_max);
    delay(4);
    
    EEPROM.write(0,1);
  }//end of default initial
    
  readEEPROM();
}

void readEEPROM(){  
  uint8_t temp_azim = EEPROM.read(1);
  delay(4);
  initAzim = EEPROM.read(2);
  delay(4);  
  initElev = EEPROM.read(3);
  delay(4);
  if(temp_azim == 1) initAzim *= -1;
  initElev *= -1;
  
  poten_azim_min = EEPROM.read(4);
  delay(4);  
  poten_azim_max = EEPROM.read(5);
  delay(4);
  poten_elev_min = EEPROM.read(6);
  delay(4);  
  poten_elev_max = EEPROM.read(7);
  delay(4);
}
