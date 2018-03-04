static void nunchuck_init(){ 
  Wire.beginTransmission(0x52);
  Wire.write(0xF0);
  Wire.write(0x55);
  Wire.endTransmission();
    
  Wire.beginTransmission(0x52);
  Wire.write(0xFB);
  Wire.write(0x00);
  Wire.endTransmission();
}

void nunchunk_update(){
  int v;
  Wire.beginTransmission(0x52);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.requestFrom(0x52, 6);

  while(Wire.available()) {
    nunchuck_buf[v] = Wire.read();
    v++;
  }
}

static int nunchuck_zbutton(){
    return ((nunchuck_buf[5] >> 0) & 1) ? 0 : 1;
}
static int nunchuck_cbutton(){
    return ((nunchuck_buf[5] >> 1) & 1) ? 0 : 1;
}

