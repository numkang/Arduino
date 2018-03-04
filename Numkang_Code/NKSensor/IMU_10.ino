/********************************** L3GD20 ********************************/

#if defined(L3GD20) && defined (GYROSCOPE)
void gyro_init() {
  delay(100);
  i2c_writeReg(L3GD20_ADDRESS ,0x20 ,0x8F ); // CTRL_REG1   380Hz ODR, 20hz filter, run!
  delay(5);
  i2c_writeReg(L3GD20_ADDRESS ,0x24 ,0x02 ); // CTRL_REG5   low pass filter enable
}

void gyro_getdata() {
  i2c_getSixRawADC(L3GD20_ADDRESS, 0x80|0x28);
  GYRO_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])/20  ,
                    ((rawADC[3]<<8) | rawADC[2])/20  ,
                    ((rawADC[5]<<8) | rawADC[4])/20  );
  gyro_remove_offset();
}
#endif

/********************************** LSM303DLHC ********************************/

#if defined(LSM303DLHC) && defined (ACCELEROMETER) && defined (MAGNETOMETER)
void acc_init() {
  delay(10);
  i2c_writeReg(LSM303DLHC_ACC_ADDRESS, 0x20, 0x27);
  i2c_writeReg(LSM303DLHC_ACC_ADDRESS, 0x23, 0x08);
}

void acc_getdata() {  
  i2c_getSixRawADC(LSM303DLHC_ACC_ADDRESS, 0x28|(1 << 7)); //combine high and low bytes, then shift right to discard lowest 4 bits (which are meaningless)
  ACC_ORIENTATION( ((int16_t)(rawADC[1]<<8) | rawADC[0]) >> 4,
                   ((int16_t)(rawADC[3]<<8) | rawADC[2]) >> 4,
                   ((int16_t)(rawADC[5]<<8) | rawADC[4]) >> 4 );
  acc_remove_offset();
}

void mag_init(){
  //i2c_writeReg(LSM303DLHC_MAG_ADDRESS, 0x02, 0x00);
  /*i2c_writeReg(LSM303DLHC_MAG_ADDRESS, 0x00, 0x10); //ODR = 15 Hz
  delay(50);
  i2c_writeReg(LSM303DLHC_MAG_ADDRESS, 0x01, 0x60); //gain 2.5 Ga
  delay(10);
  i2c_writeReg(LSM303DLHC_MAG_ADDRESS, 0x02, 0x01); //Mode register  Single Conversion
  delay(100);
  mag_data();
  delay(10);
  magScale[ROLL]  =  1160.0 / abs(magRAW[ROLL]);
  magScale[PITCH] =  1160.0 / abs(magRAW[PITCH]);
  magScale[YAW]   =  1080.0 / abs(magRAW[YAW]);*/

  // leave test mode
  i2c_writeReg(LSM303DLHC_MAG_ADDRESS, 0x00 ,0x10); //Configuration Register A  normal operation
  i2c_writeReg(LSM303DLHC_MAG_ADDRESS, 0x01 ,0x20); //Configuration Register B  1.3 Ga gain
  i2c_writeReg(LSM303DLHC_MAG_ADDRESS, 0x02 ,0x00); //Mode register 
  
  magInit = 1;
}

void mag_data(){
  i2c_getSixRawADC(LSM303DLHC_MAG_ADDRESS, 0x03);
  MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                   ((rawADC[4]<<8) | rawADC[5]) ,
                   ((rawADC[2]<<8) | rawADC[3]) );
}
#endif

/********************************** LPS331AP ********************************/
#if defined(LPS331AP) && defined (BAROMETER)
void baro_init() {
  delay(10);
  i2c_writeReg(LPS331AP_ADDRESS, 0x20, 0xE0);
}

uint8_t baro_getdata(){
  if (currentTime < LPS331AP_deadline) return 0; 
  
  LPS331AP_deadline = currentTime+10000;
  baroPressure = i2c_LPS331AP_UP_Read();
  baroTemperature = i2c_LPS331AP_UT_Read();
  Baro_Common();
  return 1;
}

long i2c_LPS331AP_UP_Start(void)
{  
  i2c_read_reg_to_buf(LPS331AP_ADDRESS, 0x28|(1 << 7), &UP_RAW, 3);  
  return (int32_t)UP_RAW[2] << 16 | (uint16_t)UP_RAW[1] << 8 | UP_RAW[0];
}

int i2c_LPS331AP_UT_Start(void)
{
  i2c_read_reg_to_buf(LPS331AP_ADDRESS, 0x2B|(1 << 7), &UT_RAW, 2);  
  return (int16_t)UT_RAW[1] << 8 | UT_RAW[0];
}

float i2c_LPS331AP_UT_Read(void)
{
  return 4250 + (float)i2c_LPS331AP_UT_Start() / 4.80; //in 0.01 degC
}

float i2c_LPS331AP_UP_Read(void)
{
  return (float)i2c_LPS331AP_UP_Start() * 0.02473754883; //in Pa
}
#endif
