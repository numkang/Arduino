//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                               EEPROM function                            //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

/*
0 for check
1-6 for acc cal
7-45 for PID tuning
46 for aircraft configuration
*/

#include <EEPROM.h>

void initEEPROM(){
  uint8_t check1 = EEPROM.read(0);
  if(check1 != 1){  
    /////// DEFAULT Gain ///////
    AKp[ROLL] = 90;
    AKp[PITCH] = 90;
  
    AKi[ROLL] = 10;
    AKi[PITCH] = 10;
  
    AKd[ROLL] = 100;
    AKd[PITCH] = 100;
    
    GKp[ROLL] = 33;
    GKp[PITCH] = 33;
    GKp[YAW] = 68;
  
    GKi[ROLL] = 30;
    GKi[PITCH] = 30;
    GKi[YAW] = 45;
  
    GKd[ROLL] = 23;
    GKd[PITCH] = 23;
    GKd[YAW] = 0;
    
    AltKp = 64;
    AltKi = 25;
    AltKd = 24;       
    
    f.Aircraft_Config = 0;
        
    for(uint8_t axis = 0; axis < 3 ; axis++){
      if(axis < 2){
        EEPROM.write(7+axis,AKp[axis]);
        delay(4);
        EEPROM.write(9+axis,AKi[axis]);
        delay(4);
        EEPROM.write(11+axis,AKd[axis]);
        delay(4);
      }
      EEPROM.write(13+axis,GKp[axis]);
      delay(4);
      EEPROM.write(16+axis,GKi[axis]);
      delay(4);
      EEPROM.write(19+axis,GKd[axis]);
      delay(4);
    }
    
    EEPROM.write(22,AltKp);
    delay(4);
    EEPROM.write(23,AltKi);
    delay(4);
    EEPROM.write(24,AltKd);
    delay(4);         

    for(uint8_t k = 0; k < 6; k++){
      EEPROM.write(1+k,0);
      delay(4);
    }/// CLEAR ACC OFFSETS
    
    EEPROM.write(46,f.Aircraft_Config);
    delay(4);
    
    EEPROM.write(0,1);
  }//end of default initial
    
  readEEPROM();
}

void readEEPROM(){  
  for(uint8_t axis = 0; axis < 3; axis++){
  #if !defined (CHR_UM6)
    ACC_Zero[axis] = EEPROM.read(1+axis*2);
    uint8_t temp_1 = EEPROM.read(2+axis*2);
    if(temp_1 != 0){ACC_Zero[axis] *= -1;} //to check if that offset is negative or not 
    //because EEPROM is byte so it cannont contain negative value
  #endif
    if(axis < 2){
      AKp[axis] = EEPROM.read(7+axis);
      AKi[axis] = EEPROM.read(9+axis);
      AKd[axis] = EEPROM.read(11+axis);
    }
    GKp[axis] = EEPROM.read(13+axis);
    GKi[axis] = EEPROM.read(16+axis);
    GKd[axis] = EEPROM.read(19+axis);
  }
  
  AltKp = EEPROM.read(22);
  AltKi = EEPROM.read(23);
  AltKd = EEPROM.read(24);    
  
  f.Aircraft_Config = EEPROM.read(46);
}

//to remember acc calibration
void writeAccConfig(){
  #if !defined (CHR_UM6)
  int16_t temp_offset[3];
  for(uint8_t axis = 0; axis < 3; axis++){
    temp_offset[axis] = ACC_Zero[axis];
    if(temp_offset[axis] < 0){
      temp_offset[axis] *= -1; 
      EEPROM.write(1+axis*2,temp_offset[axis]);
      EEPROM.write(2+axis*2,1);
    } // to check if that offset is negative or not
    else if(temp_offset[axis] >= 0){
      EEPROM.write(1+axis*2,temp_offset[axis]);
      EEPROM.write(2+axis*2,0);
    }
    delay(4);
  }
  #endif
}
