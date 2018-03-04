void switch_init(){
  DDRC |= (_BV(DDC4) | _BV(DDC5));
  DDRC &= ~(_BV(DDC0) | _BV(DDC1) | _BV(DDC2) | _BV(DDC3));
  PORTC |= (_BV(PORTC4));
  PORTC &= ~(_BV(PORTC5));
}

bool read_switch(uint8_t BUTTON){
  bool val = PINC & BUTTON;
  return val;
}

bool read_LED13(){
  bool val = PINB & B00100000;
  return val;
}

