//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                         Interrupt General Function                       //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define RX_PIN_CHECK(chan)                             \
  if (mask & PCInt_RX_Pins[chan]) {                    \
    if (!(pin & PCInt_RX_Pins[chan])) {                \
      dTime = cTime - edgeTime[chan];                  \
      if (900<dTime && dTime<2200 && chan != 7) {                   \
        rcValue[chan] = dTime;                         \
      }                                                \
      else if(chan == 7){ \
        dTime /= 1000; \
        HallValue[0] = dTime; \
      } \
    }                                                  \
    else edgeTime[chan] = cTime;                       \
  }                                                    
  
void RX_PIN_MASK(){
  uint8_t mask;
  uint8_t pin;
  uint16_t cTime, dTime;
  static uint16_t edgeTime[8];
  static uint8_t PCintLast;  
  
  pin = RX_PCINT_PIN_PORT; //read PIN
  mask = pin ^ PCintLast; //indicate which bit change 
  cTime = micros(); //keep only 16 bits
  sei(); //re-enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
  PCintLast = pin; //memorize the state of all PIN7:0
   
  //write PPM to each PIN
  #if (PCINT_PIN_COUNT > 0)
    RX_PIN_CHECK(0);
  #endif
  #if (PCINT_PIN_COUNT > 1)
    RX_PIN_CHECK(1);
  #endif
  #if (PCINT_PIN_COUNT > 2)
    RX_PIN_CHECK(2);
  #endif
  #if (PCINT_PIN_COUNT > 3)
    RX_PIN_CHECK(3);
  #endif
  #if (PCINT_PIN_COUNT > 4)
    RX_PIN_CHECK(4);
  #endif
  #if (PCINT_PIN_COUNT > 5)
    RX_PIN_CHECK(5);
  #endif
  #if (PCINT_PIN_COUNT > 6)
    RX_PIN_CHECK(6);
  #endif
  #if (PCINT_PIN_COUNT > 7)
    RX_PIN_CHECK(7);
  #endif
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                       Interrupt Filtering Function                       //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  uint8_t oldSREG;
  
  oldSREG = SREG; 
  cli(); // Let's disable interrupts
  
  data = rcValue[chan]; // Let's copy the data Automically

  SREG = oldSREG; 
  sei();// Let's enable the interrupts
  return data; // We return the value correctly copied when the IRQ's where disabled
}

void computeRC() {
  static int16_t rcData4Values[RC_CHANS][4], rcDataMean[RC_CHANS];
  static uint8_t rc4ValuesIndex = 0;
  uint8_t chan,a;
  
  rc4ValuesIndex++;
  
  for(chan = 0; chan < RC_CHANS; chan++) {
    rcData4Values[chan][rc4ValuesIndex%4] = readRawRC(chan);
    rcDataMean[chan] = 0;
    for(a = 0; a < 4; a++) rcDataMean[chan] += rcData4Values[chan][a];
    rcDataMean[chan] = (rcDataMean[chan]+2)>>2;
    if(rcDataMean[chan] < rcData[chan]-3) rcData[chan] = rcDataMean[chan]+2;
    if(rcDataMean[chan] > rcData[chan]+3) rcData[chan] = rcDataMean[chan]-2;
  }
  
  computeRCcmd();
  
  /*#if defined(BAROMETER) && defined(useBARO)
    if (RC_Command[AUX1+1] > 1500) {
      if (!f.BARO_MODE) {
        f.BARO_MODE = 1;
        AltHold = EstAlt;
        initialThrottleHold = RC_Command[THR];
        errorAltitudeI = 0;
        BaroPID = 0;
        ON_LEDPIN_31
      }
    }
    else {
      f.BARO_MODE = 0;
      OFF_LEDPIN_31
    }
  #endif*/
}

void computeRCcmd(){  
  for(uint8_t chan = 0; chan < RC_CHANS; chan++) {
    if(chan > 0 && chan < 4){                                                                               
      RC_Command[chan] = min(abs(rcData[chan]-1500),500);
      if(rcData[chan] - 1500 < 0) RC_Command[chan] *= -1;
    }                                                                                                     
    else RC_Command[chan] = rcData[chan];
  }
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                    ***only for Arduino Mega 2560***                      //
//      PIN mapping : CH1   CH2   CH3   CH4   CH5   CH6   CH7   CH8         //
//                    A8    A9    A10   A11   A12   A13   A14   A15         //
//                    THR   ROLL PITCH  YAW   AUX1  AUX2  AUX3  AUX4        //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
#if defined (MEGA)

void initPCINT() {  
  DDRK &= ~(_BV(DDK0) | _BV(DDK1) | _BV(DDK2) | _BV(DDK3) | _BV(DDK4) | _BV(DDK5) | _BV(DDK6) | _BV(DDK7)); //set PORTK as INPUT
  PORTK |= (_BV(PORTK0) | _BV(PORTK1) | _BV(PORTK2) | _BV(PORTK3) | _BV(PORTK4) | _BV(PORTK5) | _BV(PORTK6) | _BV(PORTK7)); //pull-up HIGH
  cli();
  PCICR |= _BV(PCIE2); // turn on PCMSK2
  PCMSK2 |= (_BV(PCINT16) | _BV(PCINT17) | _BV(PCINT18) | _BV(PCINT19) | _BV(PCINT20) | _BV(PCINT21) | _BV(PCINT22) | _BV(PCINT23));
  sei(); 
  
  DDRB &= ~(_BV(DDB5) | _BV(DDB6)); //set PORTB as INPUT
  PORTB |= (_BV(PORTB5) | _BV(PORTB6)); //pull-up HIGH
  cli();
  PCICR |= _BV(PCIE0); // turn on PCMSK0
  PCMSK0 |= (_BV(PCINT5) | _BV(PCINT6));
  sei();
}

ISR (PCINT2_vect){
  RX_PIN_MASK();
}

#endif

