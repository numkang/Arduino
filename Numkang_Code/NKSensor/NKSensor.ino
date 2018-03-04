//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//                                                                                      //
//                                                                                      //
//                              Numkang's code for sensor                               //
//                         SOFTWARE WRITTEN BY NUMKANG_AE20                             //
//             DEPARTMENT OF AEROSPACE ENGINEERING, KASETSART UNIVERSITY                //
//                                                                                      //
//      This program is free software: you can redistribute it and/or modify            //
//      it under the terms of the GNU General Public License as published by            //
//      the Free Software Foundation, either version 3 of the License, or               //
//      any later version. see <http://www.gnu.org/licenses/>                           //
//                                                                                      //  
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
    
  /***************************    Combined IMU Boards    ********************************/
    #define GY_80
    //#define GY_81 
    //#define GY_86
    //#define GY_87
    //#define MPU9150 
    //#define HCSR04  
    //#define CRIUS_SE_v2 //Arduino Pro Mini
    //#define NANOWII //Arduino Leonardo    
    //#define AltIMU-10
    
  /***************************    Select Sensor to use    ********************************/   
    //#define useMAG 
    #define useBARO
    //#define useSONAR //only for Arduino 168/328 echo D8, trig D12      
    //#define useGPS 

  /*************************************    Speed   **************************************/
    //#define I2C_SPEED 100000L     //100kHz normal mode, this value must be used for a genuine WMP
    #define I2C_SPEED 400000L   //400kHz fast mode, it works only with some WMP clones
    
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                 Main function                            //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////    

static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;   
    
void setup() {
  Serial.begin(115200);
  //PWMinit();
  previousTime = micros();
  DDRB |= (_BV(DDB5));
  initIMU();
}

void loop() {
  
  computeIMU();
  
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;
  
  //PWMwrite();
  
  Print(250);
  //Serial.println(cycleTime);
}


