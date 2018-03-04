//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                         Gyroscope and Accelerometer MPU                  //
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
//                                Accelerometer                             //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

/********************************** ADXL345 *********************************/

#if defined(ADXL345) && defined (ACCELEROMETER)
void acc_init() {
  delay(10);
  i2c_writeReg(ADXL345_ADDRESS, 0x2D, 0x08); //  register: Power CTRL  -- value: Set measure bit 3 on
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

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                 Magnetometer                             //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

/*********************************** HMC5883 ********************************/

#if defined(HMC5883) && defined (MAGNETOMETER)
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
                   ((rawADC[4]<<8) | rawADC[5]) ,
                   ((rawADC[2]<<8) | rawADC[3]) );
}
#endif

/*********************************** AK8975 ********************************/

#if defined(AK8975) && defined (MAGNETOMETER)
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
