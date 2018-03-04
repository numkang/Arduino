//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                         Interrupt General Function                       //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define HALL_PIN_CHECK(hchan)                          \
  if (hmask & HALL_PCInt_RX_Pins[hchan]) {             \
    if (!(hpin & HALL_PCInt_RX_Pins[hchan])) {         \
      hdTime = hcTime - hedgeTime[hchan];              \
      HallValue[hchan] = hdTime;                       \
    }                                                  \
    else hedgeTime[hchan] = hcTime;                    \
  }                                                    
  
void HALL_PIN_MASK(){
  uint8_t hmask;
  uint8_t hpin;
  uint16_t hcTime, hdTime;
  static uint16_t hedgeTime[2];
  static uint8_t hPCintLast;  
  
  hpin = HALL_PCINT_PIN_PORT; //read PIN
  hmask = hpin ^ hPCintLast; //indicate which bit change 
  hcTime = millis(); //keep only 16 bits
  sei(); //re-enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
  hPCintLast = hpin; //memorize the state of all PIN7:0
   
  //write PPM to each PIN
  #if (HALL_PCINT_PIN_COUNT > 0)
    HALL_PIN_CHECK(0);
  #endif
  #if (HALL_PCINT_PIN_COUNT > 1)
    HALL_PIN_CHECK(1);
  #endif
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                    ***only for Arduino Mega 2560***                      //
//      PIN mapping : HL    HR                                              //
//                    D11   D12   A7                                        //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#if defined (MEGA)

ISR (PCINT0_vect){
  HALL_PIN_MASK();
}

#endif

