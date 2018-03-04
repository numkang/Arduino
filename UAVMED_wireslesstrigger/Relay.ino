void relay_init(){
  DDRD |= (_BV(DDD4) | _BV(DDD5) | _BV(DDD6) | _BV(DDD7));
  PORTD &= ~(_BV(PORTD4) | _BV(PORTD5) | _BV(PORTD6) | _BV(PORTD7));

  relay_off(RELAY_1);
  relay_on(RELAY_2);
  relay_off(RELAY_3);
  relay_off(RELAY_4);
  delay(500);
  relay_on(RELAY_1);
  relay_off(RELAY_2);
  relay_on(RELAY_3);
  relay_on(RELAY_4);
  delay(500);
  relay_off(RELAY_1);
  relay_off(RELAY_4);
}

void relay_on(uint8_t RELAY_NUM){
  PORTD |= RELAY_NUM;
}

void relay_off(uint8_t RELAY_NUM){
  PORTD &= ~(RELAY_NUM);
}

