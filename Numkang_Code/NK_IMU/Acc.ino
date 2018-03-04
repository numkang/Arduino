void acc_init() {
  delay(10);
  i2c_writeReg(ADXL345_ADDRESS, 0x2D, 1<<3); //  register: Power CTRL  -- value: Set measure bit 3 on
  i2c_writeReg(ADXL345_ADDRESS, 0x31, 0x0B); //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
  i2c_writeReg(ADXL345_ADDRESS, 0x2C, 0x09); //  register: BW_RATE     -- value: rate=50hz, bw=20hz
}

void acc_getdata() {
  
  rawADC[0] = readI2Cdata(ADXL345_ADDRESS,0x32);
  rawADC[1] = readI2Cdata(ADXL345_ADDRESS,0x33);

  rawADC[2] = readI2Cdata(ADXL345_ADDRESS,0x34);
  rawADC[3] = readI2Cdata(ADXL345_ADDRESS,0x35);
 
  rawADC[4] = readI2Cdata(ADXL345_ADDRESS,0x36);
  rawADC[5] = readI2Cdata(ADXL345_ADDRESS,0x37); 
  
  ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0]) ,
                   ((rawADC[3]<<8) | rawADC[2]) ,
                   ((rawADC[5]<<8) | rawADC[4]));
  acc_remove_offset();
}

void acc_remove_offset(){
  static int32_t acc_sample[3]; 
  uint8_t axis;
  if (ACC_SAMPLE>0) {
    for (axis = 0; axis < 3; axis++) {
      if (ACC_SAMPLE == ACC_TO_SAMPLE) acc_sample[axis]=0;    
      acc_sample[axis] += accRAW[axis];      
      accRAW[axis]=0;
      acc_zero_offset[axis]=0;
    }
    if (ACC_SAMPLE == 1) {
      acc_zero_offset[ROLL]  = acc_sample[ROLL]/ACC_TO_SAMPLE;
      acc_zero_offset[PITCH] = acc_sample[PITCH]/ACC_TO_SAMPLE;
      acc_zero_offset[YAW]   = acc_sample[YAW]/ACC_TO_SAMPLE;// - ACC_1G;
      accCalibrated = 1;      
    }
    ACC_SAMPLE--;
  }
  for (axis = 0; axis < 3; axis++) {
  accRAW[axis]  -=  acc_zero_offset[axis];
  }
}
