//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                I2C initialize                            //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void initIMU(){
  i2c_init();  
  delay(100);
  
  #if defined(GYROSCOPE)
    gyro_init();
    delay(10);
  #endif
  
  #if defined(ACCELEROMETER)
    acc_init();
    delay(10);
    acc_25deg = acc_1G * 0.423;
  #endif
  
  #if defined(MAGNETOMETER)  && defined(useMAG)
    mag_init();
    delay(10);
  #endif
  
  #if defined(BAROMETER) && defined(useBARO)
    baro_init();
    delay(10);
    calibratingB = 200;  // 10 seconds init_delay + 200 * 25 ms = 15 seconds before ground pressure settles
  #endif  
  
  #if defined(ULTRASONIC) && defined(useSONAR)
    sonar_init();
    delay(10);
  #endif  
  
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                             I2C general function                         //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void i2c_init(void) {
  TWSR = 0;                                    // no prescaler => prescaler = 1
  TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;       // change the I2C clock rate
  TWCR = 1<<TWEN;                              // enable twi module, no interrupt
}

void i2c_rep_start(uint8_t address) {
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) ; // send REPEAT START condition
  waitTransmissionI2C();                       // wait until transmission completed
  TWDR = address;                              // send device address
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();                       // wail until transmission completed
}

void i2c_stop(void) {
  TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
  //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

void i2c_write(uint8_t data ) {
  TWDR = data;                                 // send data to the previously addressed device
  TWCR = (1<<TWINT) | (1<<TWEN);
  waitTransmissionI2C();
}

uint8_t i2c_read(uint8_t ack) {
  TWCR = (1<<TWINT) | (1<<TWEN) | (ack? (1<<TWEA) : 0);
  waitTransmissionI2C();
  uint8_t r = TWDR;
  if (!ack) i2c_stop();
  return r;
}

uint8_t i2c_readAck() {
  return i2c_read(1);
}

uint8_t i2c_readNak(void) {
  return i2c_read(0);
}

void waitTransmissionI2C() {
  uint16_t count = 255;
  while (!(TWCR & (1<<TWINT))) {
    count--;
    if (count==0) {              //we are in a blocking state => we don't insist
      TWCR = 0;                  //and we force a reset on TWINT register
      neutralizeTime = micros(); //we take a timestamp here to neutralize the value during a short delay
      i2c_errors_count++;
      break;
    }
  }
}

size_t i2c_read_to_buf(uint8_t add, void *buf, size_t size) {
  i2c_rep_start((add<<1) | 1);  // I2C read direction
  size_t bytes_read = 0;
  uint8_t *b = (uint8_t*)buf;
  while (size--) {
    *b++ = i2c_read(size > 0);
    bytes_read++;
  }
  return bytes_read;
}

size_t i2c_read_reg_to_buf(uint8_t add, uint8_t reg, void *buf, size_t size) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  return i2c_read_to_buf(add, buf, size);
}

void swap_endianness(void *buf, size_t size) {
  uint8_t tray;
  uint8_t *from;
  uint8_t *to;
  for (from = (uint8_t*)buf, to = &from[size-1]; from < to; from++, to--) {
    tray = *from;
    *from = *to;
    *to = tray;
  }
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg) {
  i2c_read_reg_to_buf(add, reg, &rawADC, 6);
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val) {
  i2c_rep_start(add<<1); // I2C write direction
  i2c_write(reg);        // register selection
  i2c_write(val);        // value to write in register
  i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg) {
  uint8_t val;
  i2c_read_reg_to_buf(add, reg, &val, 1);
  return val;
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           remove offset function                         //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#if defined(ACCELEROMETER)
void acc_remove_offset(){
  static int32_t acc_sample[3]; 
  uint8_t axis;
  if (ACC_SAMPLE>0) {
    for (axis = 0; axis < 3; axis++) {
      if (ACC_SAMPLE == ACC_TO_SAMPLE) acc_sample[axis]=0;    
      acc_sample[axis] +=accRAW[axis];      
      accRAW[axis]=0;
      ACCEL.zero_offset[axis]=0;
    }
    if (ACC_SAMPLE == 1) {
      ACCEL.zero_offset[ROLL]  = acc_sample[ROLL]/ACC_TO_SAMPLE;
      ACCEL.zero_offset[PITCH] = acc_sample[PITCH]/ACC_TO_SAMPLE;
      ACCEL.zero_offset[YAW]   = acc_sample[YAW]/ACC_TO_SAMPLE - ACC_1G;
      f.accCalibrated = 1;      
    }
    ACC_SAMPLE--;
  }
  for (axis = 0; axis < 3; axis++) {
  accRAW[axis]  -=  ACCEL.zero_offset[axis];
  }
}
#endif

#if defined(GYROSCOPE)
void gyro_remove_offset() {
  static int16_t lastGyro[3] = {0,0,0};
  static int32_t gyro_sample[3];
  uint8_t axis;

  if (GYRO_SAMPLE>0) {
    for (axis = 0; axis < 3; axis++) {
      if (GYRO_SAMPLE == GYRO_TO_SAMPLE) gyro_sample[axis]=0;
      gyro_sample[axis] += gyroRAW[axis];
      gyroRAW[axis]=0;
      GYRO.zero_offset[axis]=0;
      if (GYRO_SAMPLE == 1) {
        GYRO.zero_offset[axis] = gyro_sample[axis]/GYRO_TO_SAMPLE;
        f.gyroCalibrated = 1;    
      }
    }
    GYRO_SAMPLE--;
  }

  for (axis = 0; axis < 3; axis++) {
    gyroRAW[axis]  -= GYRO.zero_offset[axis];
    gyroRAW[axis] = constrain(gyroRAW[axis],lastGyro[axis]-900,lastGyro[axis]+900);  
    lastGyro[axis] = gyroRAW[axis];
  }
}
#endif

#if defined(MAGNETOMETER)
  #if !defined(MPU6050_I2C_AUX_MASTER)
    void Device_Mag_getADC(){
      mag_data();
    }
  #endif

  void mag_getdata() {
    static uint32_t t,tCal = 0;
    static int16_t magZeroTempMin[3];
    static int16_t magZeroTempMax[3];
    uint8_t axis;
    if ( currentTime < t ) return; //each read is spaced by 100ms
    t = currentTime + 100000;
    
    Device_Mag_getADC();
    
    magRAW[ROLL]  = magRAW[ROLL]  * magScale[ROLL];
    magRAW[PITCH] = magRAW[PITCH] * magScale[PITCH];
    magRAW[YAW]   = magRAW[YAW]   * magScale[YAW];
    
    if (f.magCalibrated) {
      tCal = t;
      for(axis=0;axis<3;axis++) {
        MAG.zero_offset[axis] = 0;
        magZeroTempMin[axis] = magRAW[axis];
        magZeroTempMax[axis] = magRAW[axis];
      }
      f.magCalibrated = 0;
    }
    if (magInit) { // we apply offset only once mag calibration is done
      magRAW[ROLL]  -= MAG.zero_offset[ROLL];
      magRAW[PITCH] -= MAG.zero_offset[PITCH];
      magRAW[YAW]   -= MAG.zero_offset[YAW];
    }
 
    if (tCal != 0) {
      if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
        for(axis=0; axis<3; axis++) {
          if (magRAW[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magRAW[axis];
          if (magRAW[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magRAW[axis];
        }
      } else {
        tCal = 0;
        for(axis=0; axis<3; axis++)
          MAG.zero_offset[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])/2;
      }
    }
  }
#endif

#if defined(BAROMETER)
  void Baro_Common() {
    static int32_t baroHistTab[BARO_TAB_SIZE];
    static uint8_t baroHistIdx;
  
    uint8_t indexplus1 = (baroHistIdx + 1);
    if (indexplus1 == BARO_TAB_SIZE) indexplus1 = 0;
    baroHistTab[baroHistIdx] = baroPressure;
    baroPressureSum += baroHistTab[baroHistIdx];
    baroPressureSum -= baroHistTab[indexplus1];
    baroHistIdx = indexplus1;  
  }
#endif
