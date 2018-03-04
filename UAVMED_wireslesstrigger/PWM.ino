void PWM_init(){
  DDRB |= (_BV(DDB1) | _BV(DDB2));
  TCCR1A |= (_BV(COM1A1) | _BV(COM1B1));

  PWM_write(CH7_MotorInterlock, PWM_LOW);
  PWM_write(CH8_Brake, PWM_LOW);

  DDRB &= ~(_BV(DDB0) | _BV(DDB3));
  PORTB |= (_BV(PORTB0) | _BV(PORTB3));
  cli();
  PCICR |= _BV(PCIE0); // turn on PCMSK0
  PCMSK0 |= (_BV(PCINT0) | _BV(PCINT3));
  sei();
}

void PWM_write(uint8_t chan, uint16_t PWM){
  if(chan == PWM_Pin9){
    OCR1A = PWM >> 3;
  }
  else if(chan == PWM_Pin10){
    OCR1B = PWM >> 3;
  }
}

#define RX_PIN_CHECK(chan)                             \
  if (mask & PCInt_RX_Pins[chan]) {                    \
    if (!(pin & PCInt_RX_Pins[chan])) {                \
      dTime = cTime - edgeTime[chan];                  \
      if (800<dTime && dTime<2200) {                   \
        rcValue[chan] = dTime;                         \
      }                                                \
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
}

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
    RC_Command[chan] = rcData[chan];
  }
}

ISR (PCINT0_vect){
  RX_PIN_MASK();
}

