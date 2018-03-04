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

//////////////////////////////////////////////////////////////////////////////////////////

void lla2ecef(float lat,float lon,float alt){
  lat /= 10000000;
  lon /= 10000000;
  alt /= 10;
  float N;
  //intermediate calculation
  //(prime vertical radius of curvature)
  N = a / sqrt(1 - e*e * sin(lat)*sin(lat));
  //results:
  GPS_ecef[X] = (N+alt) * cos(lat) * cos(lon);
  GPS_ecef[Y] = (N+alt) * cos(lat) * sin(lon);
  GPS_ecef[Z] = ((1-e*e) * N + alt) * sin(lat);
}

//////////////////////////////////////////////////////////////////////////////////////////

/*void ecef2lla(int32_t x,int32_t y,int32_t z)

  //calculations:
  b   = sqrt(a^2*(1-e^2));
  ep  = sqrt((a^2-b^2)/b^2);
  p   = sqrt(x.^2+y.^2);
  th  = atan2(a*z,b*p);
  lon = atan2(y,x);
  lat = atan2((z+ep^2.*b.*sin(th).^3),(p-e^2.*a.*cos(th).^3));
  N   = a./sqrt(1-e^2.*sin(lat).^2);
  alt = p./cos(lat)-N;

  //return lon in range [0,2*pi)
  lon = mod(lon,2*pi);

  //correct for numerical instability in altitude near exact poles:
  //(after this correction, error is about 2 millimeters, which is about
  //the same as the numerical precision of the overall function)

  k=abs(x)<1 & abs(y)<1;
  alt(k) = abs(z(k))-b;
}*/

////////////////////////////////////////////////////////////////////////////////////////
void hextochar(uint8_t val){
  uint8_t a = val >> 4;
  if (a >= 0 && a <= 9) checksum[0] = '0' + a;
  if (a >= 10 && a <= 15) checksum[0] = '7' + a;
  
  uint8_t b = val - (a << 4);
  if (b >= 0 && b <= 9) checksum[1] = '0' + b;
  if (b >= 10 && b <= 15) checksum[1] = '7' + b;
}
