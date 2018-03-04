void initPWM(){
pinMode(2,OUTPUT);
pinMode(3,OUTPUT);

TCCR3A &= ~(1<<WGM30); 
TCCR3A |= _BV(WGM31);
TCCR3B &= ~(1<<CS31);
TCCR3B |= _BV(WGM33);
ICR3 |= 0x3FFF;//16383
TCCR3A |= _BV(COM3B1) | _BV(COM3C1);
OCR3B = PWMscale(1000);
OCR3C = PWMscale(1000);
}

void updateDutyCycle() {
pPWM[M1] = constrain(pPWM[M1],1000,2000);
pPWM[M2] = constrain(pPWM[M2],1000,2000);
OCR3B = PWMscale(pPWM[M1]);
OCR3C = PWMscale(pPWM[M2]);
}
