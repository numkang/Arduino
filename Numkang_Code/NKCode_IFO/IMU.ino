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
  #endif
  
  #if defined(MAGNETOMETER) && defined(useMAG)
    mag_init();
    delay(10);
  #endif
  
  #if defined(BAROMETER) && defined(useBARO)
    baro_init();
    delay(10);
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
#if !defined (CHR_UM6)
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
#endif
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                         Gyroscope and Accelerometer IMU                  //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

/********************************** MPU6050 *********************************/

#if defined(MPU6050) && defined (ACCELEROMETER) && defined (GYROSCOPE)
void gyro_init() {
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(5);
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_writeReg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  // enable I2C bypass for AUX I2C
  #if defined(MAGNETOMETER)
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x02);           //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
  #endif
}

void gyro_getdata () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x43);
  GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>2 , // range: +/- 8192; +/- 2000 deg/sec
                    ((rawADC[2]<<8) | rawADC[3])>>2 ,
                    ((rawADC[4]<<8) | rawADC[5])>>2 );
  gyro_remove_offset();
}

void acc_init () {
  i2c_writeReg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
  //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480

  #if defined(MPU6050_I2C_AUX_MASTER)
    //at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
    //now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
    i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
    i2c_writeReg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
    i2c_writeReg(MPU6050_ADDRESS, 0x25, 0x80|MAG_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
    i2c_writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
    i2c_writeReg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)
  #endif
}

void acc_getdata () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x3B);
  ACC_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>3 ,
                   ((rawADC[2]<<8) | rawADC[3])>>3 ,
                   ((rawADC[4]<<8) | rawADC[5])>>3 );
  acc_remove_offset();
}

//The MAG acquisition function must be replaced because we now talk to the MPU device
  #if defined(MPU6050_I2C_AUX_MASTER)
    void Device_Mag_getADC() {
      i2c_getSixRawADC(MPU6050_ADDRESS, 0x49);               //0x49 is the first memory room for EXT_SENS_DATA
      #if defined(HMC5843)
        MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                         ((rawADC[2]<<8) | rawADC[3]) ,
                         ((rawADC[4]<<8) | rawADC[5]) );
      #endif
      #if defined (HMC5883)  
        MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                         ((rawADC[4]<<8) | rawADC[5]) ,
                         ((rawADC[2]<<8) | rawADC[3]) );
      #endif
      #if defined (MAG3110)
        MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,          
                         ((rawADC[2]<<8) | rawADC[3]) ,     
                         ((rawADC[4]<<8) | rawADC[5]) );
      #endif
      #if defined (AK8975)
        MAG_ORIENTATION( ((rawADC[1]<<8) | rawADC[0]) ,          
                         ((rawADC[3]<<8) | rawADC[2]) ,     
                         ((rawADC[5]<<8) | rawADC[4]) );
        //Start another meassurement
        i2c_writeReg(MAG_ADDRESS,0x0a,0x01);
      #endif
    }
  #endif
#endif

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                      Accelerometer and Magnetometer IMU                  //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

/********************************** LSM303DLHC ********************************/

#if defined(LSM303DLHC) && defined (ACCELEROMETER) && defined (MAGNETOMETER)
void acc_init() {
  delay(10);
  i2c_writeReg(LSM303DLHC_ACC_ADDRESS, 0x20, 0x77); //400hz
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
  i2c_writeReg(LSM303DLHC_MAG_ADDRESS, 0x00, 0x10); //ODR = 15 Hz
  delay(50);
  i2c_writeReg(LSM303DLHC_MAG_ADDRESS, 0x01, 0x60); //gain 2.5 Ga
  delay(10);
  i2c_writeReg(LSM303DLHC_MAG_ADDRESS, 0x02, 0x01); //Mode register  Single Conversion
  delay(100);
  mag_data();
  delay(10);
  magScale[ROLL]  =  1160.0 / abs(magRAW[ROLL]);
  magScale[PITCH] =  1160.0 / abs(magRAW[PITCH]);
  magScale[YAW]   =  1080.0 / abs(magRAW[YAW]);

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

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                Accelerometer                             //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

/********************************** ADXL345 *********************************/

#if defined(ADXL345) && defined (ACCELEROMETER)
void acc_init() {
  delay(10);
  i2c_writeReg(ADXL345_ADDRESS, 0x2D, 1<<3); //  register: Power CTRL  -- value: Set measure bit 3 on
  i2c_writeReg(ADXL345_ADDRESS, 0x31, 0x0B); //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
  i2c_writeReg(ADXL345_ADDRESS, 0x2C, 0x09); //  register: BW_RATE     -- value: rate=50hz, bw=20hz
}

void acc_getdata() {
  i2c_getSixRawADC(ADXL345_ADDRESS, 0x32);
  ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0]) ,
                   ((rawADC[3]<<8) | rawADC[2]) ,
                   ((rawADC[5]<<8) | rawADC[4]) );
  acc_remove_offset();
}
#endif

/*********************************** BMA180 *********************************/

#if defined(BMA180) && defined (ACCELEROMETER)
void acc_init(){
  delay(10);
  //default range 2G: 1G = 4096 unit.
  i2c_writeReg(BMA180_ADDRESS,0x0D,1<<4); // register: ctrl_reg0  -- value: set bit ee_w to 1 to enable writing
  delay(5);
  uint8_t control = i2c_readReg(BMA180_ADDRESS, 0x20);
  control = control & 0x0F;        // save tcs register
  //control = control | (0x01 << 4); // register: bw_tcs reg: bits 4-7 to set bw -- value: set low pass filter to 20Hz
  control = control | (0x00 << 4); // set low pass filter to 10Hz (bits value = 0000xxxx)
  i2c_writeReg(BMA180_ADDRESS, 0x20, control);
  delay(5);
  control = i2c_readReg(BMA180_ADDRESS, 0x30);
  control = control & 0xFC;        // save tco_z register
  control = control | 0x00;        // set mode_config to 0
  i2c_writeReg(BMA180_ADDRESS, 0x30, control);
  delay(5); 
  control = i2c_readReg(BMA180_ADDRESS, 0x35);
  control = control & 0xF1;        // save offset_x and smp_skip register
  control = control | (0x05 << 1); // set range to 8G
  i2c_writeReg(BMA180_ADDRESS, 0x35, control);
  delay(5); 
}

void acc_getdata() {
  i2c_getSixRawADC(BMA180_ADDRESS, 0x02);
  //usefull info is on the 14 bits  [2-15] bits  /4 => [0-13] bits  /4 => 12 bit resolution
  ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])>>4 ,
                   ((rawADC[3]<<8) | rawADC[2])>>4 ,
                   ((rawADC[5]<<8) | rawADC[4])>>4 );
  acc_remove_offset();
}
#endif

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                  Gyroscope                               //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

/********************************** L3G4200D ********************************/

#if defined(L3G4200D) && defined (GYROSCOPE)
void gyro_init() {
  delay(100);
  i2c_writeReg(L3G4200D_ADDRESS ,0x20 ,0x8F ); // CTRL_REG1   400Hz ODR, 20hz filter, run!
  delay(5);
  i2c_writeReg(L3G4200D_ADDRESS ,0x24 ,0x02 ); // CTRL_REG5   low pass filter enable
}

void gyro_getdata() {
  i2c_getSixRawADC(L3G4200D_ADDRESS, 0x80|0x28);
  GYRO_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])/20  ,
                    ((rawADC[3]<<8) | rawADC[2])/20  ,
                    ((rawADC[5]<<8) | rawADC[4])/20  );
  gyro_remove_offset();
}
#endif

/********************************** ITG3205 ********************************/

#if defined(ITG3205) && defined (GYROSCOPE)
void gyro_init() {
  delay(100);
  i2c_writeReg(ITG3205_ADDRESS, 0x3E, 0x80); //register: Power Management  --  value: reset device
  //delay(5);
  //i2c_writeReg(ITG3205_ADDRESS, 0x15, ITG3200_SMPLRT_DIV); //register: Sample Rate Divider  -- default value = 0: OK
  delay(5);
  i2c_writeReg(ITG3205_ADDRESS, 0x16, 0x18 + ITG3205_DLPF_CFG); //register: DLPF_CFG - low pass filter configuration
  delay(5);
  i2c_writeReg(ITG3205_ADDRESS, 0x3E, 0x03); //register: Power Management  --  value: PLL with Z Gyro reference
  delay(100);
}

void gyro_getdata(){
  i2c_getSixRawADC(ITG3205_ADDRESS, 0x1D);
  GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>2 , // range: +/- 8192; +/- 2000 deg/sec
                    ((rawADC[2]<<8) | rawADC[3])>>2 ,
                    ((rawADC[4]<<8) | rawADC[5])>>2 );
  gyro_remove_offset();
}
#endif

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

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                 Magnetometer                             //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

/*********************************** HMC5883 ********************************/

#if defined(HMC5883) && defined (MAGNETOMETER) && defined(useMAG)
void mag_init(){
  i2c_writeReg(HMC5883_ADDRESS, 0x00, 0x71); //Configuration Register A  positive bias, ODR = 15 Hz
  delay(50);
  i2c_writeReg(HMC5883_ADDRESS, 0x01, 0x60); //Configuration Register B  gain 2.5 Ga
  delay(10);
  i2c_writeReg(HMC5883_ADDRESS, 0x02, 0x01); //Mode register  Single Conversion
  delay(100);
  mag_data();
  delay(10);
  magScale[ROLL]  =  1160.0 / abs(magRAW[ROLL]);
  magScale[PITCH] =  1160.0 / abs(magRAW[PITCH]);
  magScale[YAW]   =  1080.0 / abs(magRAW[YAW]);

  // leave test mode
  i2c_writeReg(HMC5883_ADDRESS, 0x00 ,0x70); //Configuration Register A  normal operation
  i2c_writeReg(HMC5883_ADDRESS, 0x01 ,0x20); //Configuration Register B  1.3 Ga gain
  i2c_writeReg(HMC5883_ADDRESS, 0x02 ,0x00); //Mode register 
  
  magInit = 1;
}

void mag_data(){
  i2c_getSixRawADC(HMC5883_ADDRESS, 0x03);
  MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                   ((rawADC[2]<<8) | rawADC[3]) ,
                   ((rawADC[4]<<8) | rawADC[5]) );
}
#endif

/*********************************** AK8975 ********************************/

#if defined(AK8975) && defined (MAGNETOMETER) && defined(useMAG)
void mag_init() {
    delay(100);
    i2c_writeReg(MAG_ADDRESS,0x0a,0x01);  //Start the first conversion
    delay(100);
    magInit = 1;
  }

void mag_data() {
  i2c_getSixRawADC(MAG_ADDRESS, MAG_DATA_REGISTER);
  MAG_ORIENTATION( ((rawADC[1]<<8) | rawADC[0]) ,          
                   ((rawADC[3]<<8) | rawADC[2]) ,     
                   ((rawADC[5]<<8) | rawADC[4]) );
  //Start another meassurement
  i2c_writeReg(MAG_ADDRESS,0x0a,0x01);
}
#endif

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                  Barometer                               //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

/*********************************** BMP085 ********************************/

#if defined(BMP085) && defined(BAROMETER) && defined(useBARO)
static struct {
  // sensor registers from the BOSCH BMP085 datasheet
  int16_t  ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t  b1, b2, mb, mc, md;
  union {uint16_t val; uint8_t raw[2]; } ut; //uncompensated T
  union {uint32_t val; uint8_t raw[4]; } up; //uncompensated P
  uint8_t  state;
  uint32_t deadline;
} bmp085_ctx;  
#define OSS 3

void i2c_BMP085_readCalibration(){
  delay(10);
  //read calibration data in one go
  size_t s_bytes = (uint8_t*)&bmp085_ctx.md - (uint8_t*)&bmp085_ctx.ac1 + sizeof(bmp085_ctx.ac1);
  i2c_read_reg_to_buf(BMP085_ADDRESS, 0xAA, &bmp085_ctx.ac1, s_bytes);
  // now fix endianness
  int16_t *p;
  for (p = &bmp085_ctx.ac1; p <= &bmp085_ctx.md; p++) {
    swap_endianness(p, sizeof(*p));
  }
}

void baro_init() {
  delay(10);
  i2c_BMP085_readCalibration();
  delay(5);
  i2c_BMP085_UT_Start(); 
  bmp085_ctx.deadline = currentTime+5000;
}

// read uncompensated temperature value: send command first
void i2c_BMP085_UT_Start() {
  i2c_writeReg(BMP085_ADDRESS,0xf4,0x2e);
  i2c_rep_start(BMP085_ADDRESS<<1);
  i2c_write(0xF6);
  i2c_stop();
}

// read uncompensated pressure value: send command first
void i2c_BMP085_UP_Start () {
  i2c_writeReg(BMP085_ADDRESS,0xf4,0x34+(OSS<<6)); // control register value for oversampling setting 3
  i2c_rep_start(BMP085_ADDRESS<<1); //I2C write direction => 0
  i2c_write(0xF6);
  i2c_stop();
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
void i2c_BMP085_UP_Read () {
  i2c_rep_start((BMP085_ADDRESS<<1) | 1);//I2C read direction => 1
  bmp085_ctx.up.raw[2] = i2c_readAck();
  bmp085_ctx.up.raw[1] = i2c_readAck();
  bmp085_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
void i2c_BMP085_UT_Read() {
  i2c_rep_start((BMP085_ADDRESS<<1) | 1);//I2C read direction => 1
  bmp085_ctx.ut.raw[1] = i2c_readAck();
  bmp085_ctx.ut.raw[0] = i2c_readNak();
}

void i2c_BMP085_Calculate() {
  int32_t  x1, x2, x3, b3, b5, b6, p, tmp;
  uint32_t b4, b7;
  // Temperature calculations
  x1 = ((int32_t)bmp085_ctx.ut.val - bmp085_ctx.ac6) * bmp085_ctx.ac5 >> 15;
  x2 = ((int32_t)bmp085_ctx.mc << 11) / (x1 + bmp085_ctx.md);
  b5 = x1 + x2;
  baroTemperature = (b5 * 10 + 8) >> 4; // in 0.01 degC (same as MS561101BA temperature)
  // Pressure calculations
  b6 = b5 - 4000;
  x1 = (bmp085_ctx.b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = bmp085_ctx.ac2 * b6 >> 11;
  x3 = x1 + x2;
  tmp = bmp085_ctx.ac1;
  tmp = (tmp*4 + x3) << OSS;
  b3 = (tmp+2)/4;
  x1 = bmp085_ctx.ac3 * b6 >> 13;
  x2 = (bmp085_ctx.b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (bmp085_ctx.ac4 * (uint32_t)(x3 + 32768)) >> 15;
  b7 = ((uint32_t) (bmp085_ctx.up.val >> (8-OSS)) - b3) * (50000 >> OSS);
  p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  baroPressure = p + ((x1 + x2 + 3791) >> 4);
}

//return 0: no data available, no computation ;  1: new value available  ; 2: no new value, but computation time
uint8_t baro_getdata() {                   // first UT conversion is started in init procedure
  if (currentTime < bmp085_ctx.deadline) return 0; 
  bmp085_ctx.deadline = currentTime+6000; // 1.5ms margin according to the spec (4.5ms T convetion time)
  if (bmp085_ctx.state == 0) {
    i2c_BMP085_UT_Read(); 
    i2c_BMP085_UP_Start(); 
    bmp085_ctx.state = 1; 
    Baro_Common();
    bmp085_ctx.deadline += 21000;   // 6000+21000=27000 1.5ms margin according to the spec (25.5ms P convetion time with OSS=3)
    return 1;
  } else {
    i2c_BMP085_UP_Read(); 
    i2c_BMP085_UT_Start(); 
    i2c_BMP085_Calculate(); 
    bmp085_ctx.state = 0; 
    return 2;
  }
}
#endif

/*********************************** MS561101BA ********************************/

#if defined(MS561101BA) && defined(BAROMETER) && defined(useBARO)

// registers of the device
#define MS561101BA_PRESSURE    0x40
#define MS561101BA_TEMPERATURE 0x50
#define MS561101BA_RESET       0x1E

// OSR (Over Sampling Ratio) constants
#define MS561101BA_OSR_256  0x00
#define MS561101BA_OSR_512  0x02
#define MS561101BA_OSR_1024 0x04
#define MS561101BA_OSR_2048 0x06
#define MS561101BA_OSR_4096 0x08

#define OSR MS561101BA_OSR_4096

static struct {
  // sensor registers from the MS561101BA datasheet
  uint16_t c[7];
  union {uint32_t val; uint8_t raw[4]; } ut; //uncompensated T
  union {uint32_t val; uint8_t raw[4]; } up; //uncompensated P
  uint8_t  state;
  uint32_t deadline;
} ms561101ba_ctx;

void i2c_MS561101BA_reset(){
  i2c_writeReg(MS561101BA_ADDRESS, MS561101BA_RESET, 0);
}

void i2c_MS561101BA_readCalibration(){
  union {uint16_t val; uint8_t raw[2]; } data;
  for(uint8_t i=0;i<6;i++) {
    i2c_rep_start(MS561101BA_ADDRESS<<1);
    i2c_write(0xA2+2*i);
    delay(10);
    i2c_rep_start((MS561101BA_ADDRESS<<1) | 1);//I2C read direction => 1
    delay(10);
    data.raw[1] = i2c_readAck();  // read a 16 bit register
    data.raw[0] = i2c_readNak();
    ms561101ba_ctx.c[i+1] = data.val;
  }
}

void  baro_init() {
  delay(10);
  i2c_MS561101BA_reset();
  delay(100);
  i2c_MS561101BA_readCalibration();
  delay(10);
  i2c_MS561101BA_UT_Start(); 
  ms561101ba_ctx.deadline = currentTime+10000; 
}

// read uncompensated temperature value: send command first
void i2c_MS561101BA_UT_Start() {
  i2c_rep_start(MS561101BA_ADDRESS<<1);      // I2C write direction
  i2c_write(MS561101BA_TEMPERATURE + OSR);  // register selection
  i2c_stop();
}

// read uncompensated pressure value: send command first
void i2c_MS561101BA_UP_Start () {
  i2c_rep_start(MS561101BA_ADDRESS<<1);      // I2C write direction
  i2c_write(MS561101BA_PRESSURE + OSR);     // register selection
  i2c_stop();
}

// read uncompensated pressure value: read result bytes
void i2c_MS561101BA_UP_Read () {
  i2c_rep_start(MS561101BA_ADDRESS<<1);
  i2c_write(0);
  i2c_rep_start((MS561101BA_ADDRESS<<1) | 1);
  ms561101ba_ctx.up.raw[2] = i2c_readAck();
  ms561101ba_ctx.up.raw[1] = i2c_readAck();
  ms561101ba_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
void i2c_MS561101BA_UT_Read() {
  i2c_rep_start(MS561101BA_ADDRESS<<1);
  i2c_write(0);
  i2c_rep_start((MS561101BA_ADDRESS<<1) | 1);
  ms561101ba_ctx.ut.raw[2] = i2c_readAck();
  ms561101ba_ctx.ut.raw[1] = i2c_readAck();
  ms561101ba_ctx.ut.raw[0] = i2c_readNak();
}

void i2c_MS561101BA_Calculate() {
  int32_t off2,sens2,delt;

  int64_t dT       = (int32_t)ms561101ba_ctx.ut.val - ((int32_t)ms561101ba_ctx.c[5] << 8);
  baroTemperature  = 2000 + ((dT * ms561101ba_ctx.c[6])>>23);
  int64_t off      = ((uint32_t)ms561101ba_ctx.c[2] <<16) + ((dT * ms561101ba_ctx.c[4]) >> 7);
  int64_t sens     = ((uint32_t)ms561101ba_ctx.c[1] <<15) + ((dT * ms561101ba_ctx.c[3]) >> 8);

  if (baroTemperature < 2000) { // temperature lower than 20st.C 
    delt = baroTemperature-2000;
    delt  = 5*delt*delt;
    off2  = delt>>1;
    sens2 = delt>>2;
    if (baroTemperature < -1500) { // temperature lower than -15st.C
      delt  = baroTemperature+1500;
      delt  = delt*delt;
      off2  += 7 * delt;
      sens2 += (11 * delt)>>1;
    }
    off  -= off2; 
    sens -= sens2;
  }

  baroPressure = (( (ms561101ba_ctx.up.val * sens ) >> 21) - off) >> 15;
}

//return 0: no data available, no computation ;  1: new value available  ; 2: no new value, but computation time
uint8_t baro_getdata() {                            // first UT conversion is started in init procedure
  if (currentTime < ms561101ba_ctx.deadline) return 0; 
  ms561101ba_ctx.deadline = currentTime+10000;  // UT and UP conversion take 8.5ms so we do next reading after 10ms
  if (ms561101ba_ctx.state == 0) {
    i2c_MS561101BA_UT_Read(); 
    i2c_MS561101BA_UP_Start(); 
    Baro_Common();                              // moved here for less timecycle spike
    ms561101ba_ctx.state = 1;
    return 1;
  } else {
    i2c_MS561101BA_UP_Read();
    i2c_MS561101BA_UT_Start(); 
    i2c_MS561101BA_Calculate();
    ms561101ba_ctx.state = 0; 
    return 2;
  }
}
#endif

/********************************** LPS331AP ********************************/
#if defined(LPS331AP) && defined (BAROMETER)
static struct {
  // sensor registers from the MS561101BA datasheet
  union {uint32_t val; uint8_t raw[2]; } ut; //uncompensated T
  union {uint32_t val; uint8_t raw[3]; } up; //uncompensated P
  uint8_t  state;
  uint32_t deadline;
} LPS331AP_ctx;

void baro_init() {
  delay(10);
  i2c_writeReg(LPS331AP_ADDRESS, 0x20, 0xE0);
}

uint8_t baro_getdata(){
  if (currentTime < LPS331AP_ctx.deadline) return 0; 
  
  LPS331AP_ctx.deadline = currentTime+10000;
  baroPressure = i2c_LPS331AP_UP_Read();
  baroTemperature = i2c_LPS331AP_UT_Read();
  Baro_Common();
  return 1;
}

long i2c_LPS331AP_UP_Start(void)
{  
  i2c_read_reg_to_buf(LPS331AP_ADDRESS, 0x28|(1 << 7), &LPS331AP_ctx.up, 3);  
  return (int32_t)LPS331AP_ctx.up.raw[2] << 16 | (uint16_t)LPS331AP_ctx.up.raw[1] << 8 | LPS331AP_ctx.up.raw[0];
}

int i2c_LPS331AP_UT_Start(void)
{
  i2c_read_reg_to_buf(LPS331AP_ADDRESS, 0x2B|(1 << 7), &LPS331AP_ctx.ut, 2);  
  return (int16_t)LPS331AP_ctx.ut.raw[1] << 8 | LPS331AP_ctx.ut.raw[0];
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

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                 Ultrasonic                               //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

/*********************************** HC-SR04 ********************************/

#if defined (HCSR04) && defined (ULTRASONIC) && defined(useSONAR)
void sonar_init(){
  DDRB |= (_BV(DDB4)); //set PORTB (D12) as an OUTPUT trigPIN
  DDRB &= ~(_BV(DDB0)); //set PORTB as INPUT D8 echoPIN
}

void sonar_getdata(){
  if(currentTime/1000 - last_time_sonar > 500){
    last_time_sonar = currentTime/1000;
    PORTB &= ~(_BV(PORTB4));
    delayMicroseconds(2);  
    PORTB |= (_BV(PORTB4));
    delayMicroseconds(10);  
    PORTB &= ~(_BV(PORTB4));  
  
    duration = pulseIn(echoPin, HIGH);
    
    distanceCM = duration/58; //in cm
    distanceINCH = duration/148; //in inch
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                     GPS                                  //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
