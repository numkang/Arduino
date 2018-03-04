//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//                                                                                      //
//                                                                                      //
//                     Numkang's code for Microcontroller                               //
//                      SOFTWARE WRITTEN BY NUMKANG_AE20                                //
//          DEPARTMENT OF AEROSPACE ENGINEERING, KASETSART UNIVERSITY                   //
//                                                                                      //
//      This program is free software: you can redistribute it and/or modify            //
//      it under the terms of the GNU General Public License as published by            //
//      the Free Software Foundation, either version 3 of the License, or               //
//      any later version. see <http://www.gnu.org/licenses/>                           //
//                                                                                      //  
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

static uint32_t currentTime = 0;
static uint32_t previousTime = 0;
static uint16_t cycleTime = 0;
static uint32_t taskTime50 = 0;

#define RC_CHANS 5
#define PCINT_PIN_COUNT 5
#define PCINT_RX_BITS (1<<2),(1<<4),(1<<5),(1<<6),(1<<7)
#define RX_PCINT_PIN_PORT PIND

static int16_t rcData[RC_CHANS]; //interval [1000;2000]
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS};
volatile uint16_t rcValue[RC_CHANS];
volatile int16_t RC_Command[RC_CHANS];

static uint8_t trig = 0;

void setup() {
  PWMinit();
  initPCINT();
  pinMode(13,OUTPUT);
  Serial.begin(115200);
}

void loop() {
  if(currentTime > taskTime50){ //50 Hz task
    taskTime50 = currentTime + 20000;    
    computeRC();        
  }
  
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime; 
  
  if(RC_Command[0] < 1100){
    if(RC_Command[1] > 1500){
      trig = 1;
    }
    else{
      trig = 0;
    }
  }
  
  digitalWrite(13,trig);
  OCR2B = RC_Command[0]>>3;
}
