#define DIGIT_TO_VAL(_x)        (_x - '0')

uint32_t GPS_coord_to_degrees(char* s) {
  char *p, *q;
  uint8_t deg = 0, min = 0;
  unsigned int frac_min = 0;
  uint8_t i;

  // scan for decimal point or end of field
  for (p = s; isdigit(*p); p++) ;
  q = s;

  // convert degrees
  while ((p - q) > 2) {
    if (deg)
      deg *= 10;
    deg += DIGIT_TO_VAL(*q++);
  }
  // convert minutes
  while (p > q) {
    if (min)
      min *= 10;
    min += DIGIT_TO_VAL(*q++);
  }
  // convert fractional minutes
  // expect up to four digits, result is in
  // ten-thousandths of a minute
  if (*p == '.') {
    q = p + 1;
    for (i = 0; i < 4; i++) {
      frac_min *= 10;
      if (isdigit(*q))
        frac_min += *q++ - '0';
    }
  }
  return deg * 10000000UL + (min * 1000000UL + frac_min*100UL) / 6;
}

//////////////////////////////////////////////////////////////////////////////////////////

// helper functions 
int16_t grab_fields(char* src, uint8_t mult) {  // convert string to uint16
  uint8_t i;
  int16_t tmp = 0;

  for(i=0; src[i]!=0; i++) {
    if(src[i] == '.') {
      i++;
      if(mult==0)   break;
      else  src[i+mult] = 0;
    }
    tmp *= 10;
    if(src[i] >='0' && src[i] <='9') tmp += src[i]-'0';
  }
  if(src[0] == '-') tmp *= -1;
  return tmp;
}

//////////////////////////////////////////////////////////////////////////////////////////

uint8_t hex_c(uint8_t n) {    // convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if(n>9)  n -= 7;
  n &= 0x0F;
  return n;
}

////////////////////////////////////////////////////////////////////////////////////////
void hextochar(uint8_t val){
  uint8_t a = val >> 4;
  if (a >= 0 && a <= 9) checksum[0] = '0' + a;
  if (a >= 10 && a <= 15) checksum[0] = '7' + a;
  
  uint8_t b = val - (a << 4);
  if (b >= 0 && b <= 9) checksum[1] = '0' + b;
  if (b >= 10 && b <= 15) checksum[1] = '7' + b;
}
