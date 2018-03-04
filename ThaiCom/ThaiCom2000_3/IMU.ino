
/********************************** MPU6050 *********************************/

void gyro_init() {
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(5);
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_writeReg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
  // enable I2C bypass for AUX I2C
  
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x02);           //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0

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

  
    //at this stage, the MAG is configured via the original MAG init function in I2C bypass mode
    //now we configure MPU as a I2C Master device to handle the MAG via the I2C AUX port (done here for HMC5883)
    i2c_writeReg(MPU6050_ADDRESS, 0x6A, 0b00100000);       //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x00);             //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
    i2c_writeReg(MPU6050_ADDRESS, 0x24, 0x0D);             //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
    i2c_writeReg(MPU6050_ADDRESS, 0x25, 0x80|MAG_ADDRESS);//I2C_SLV0_ADDR -- I2C_SLV4_RW=1 (read operation) ; I2C_SLV4_ADDR=MAG_ADDRESS
    i2c_writeReg(MPU6050_ADDRESS, 0x26, MAG_DATA_REGISTER);//I2C_SLV0_REG  -- 6 data bytes of MAG are stored in 6 registers. First register address is MAG_DATA_REGISTER
    i2c_writeReg(MPU6050_ADDRESS, 0x27, 0x86);             //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=3 (3x2 bytes)

}

void acc_getdata () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x3B);
  ACC_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>3 ,
                   ((rawADC[2]<<8) | rawADC[3])>>3 ,
                   ((rawADC[4]<<8) | rawADC[5])>>3 );
  acc_remove_offset();
}

//The MAG acquisition function must be replaced because we now talk to the MPU device
  
    void Device_Mag_getADC() {
      i2c_getSixRawADC(MPU6050_ADDRESS, 0x49);               //0x49 is the first memory room for EXT_SENS_DATA
      
        MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                         ((rawADC[4]<<8) | rawADC[5]) ,
                         ((rawADC[2]<<8) | rawADC[3]) );
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           remove offset function                         //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void acc_remove_offset(){ //MPU6050
  static int32_t acc_sample[3]; 
  uint8_t axis;
  if (calibratingA>0) {
    for (axis = 0; axis < 3; axis++) {
      if (calibratingA == ACC_TO_SAMPLE) acc_sample[axis]=0;    
      acc_sample[axis] += accRAW[axis];      
      accRAW[axis]=0;
      ACC_Zero[axis]=0;
    }
    if (calibratingA == 1) {
      /*ACC_Zero[ROLL]  = acc_sample[ROLL]/ACC_TO_SAMPLE;
      ACC_Zero[PITCH] = acc_sample[PITCH]/ACC_TO_SAMPLE;
      ACC_Zero[YAW]   = acc_sample[YAW]/ACC_TO_SAMPLE - ACC_1G;*/
      ACC_Zero[ROLL]  = 3;
      ACC_Zero[PITCH] = 13;
      ACC_Zero[YAW]   = 218 - ACC_1G;
      f.accCalibrated = 1;      
    }
    calibratingA--;
  }
  for (axis = 0; axis < 3; axis++) {
    accRAW[axis] -= ACC_Zero[axis];
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void gyro_remove_offset(){
  static int16_t lastGyro[3] = {0,0,0};
  static int32_t gyro_sample[3];
  uint8_t axis;

  if (calibratingG>0) {
    for (axis = 0; axis < 3; axis++) {
      if (calibratingG == GYRO_TO_SAMPLE) gyro_sample[axis]=0;
      gyro_sample[axis] += gyroRAW[axis];
      gyroRAW[axis]=0;
      GYRO_Zero[axis]=0;
      if (calibratingG == 1) {
        GYRO_Zero[axis] = gyro_sample[axis]/GYRO_TO_SAMPLE;
        f.gyroCalibrated = 1;    
      }
    }
    calibratingG--;
  }

  for (axis = 0; axis < 3; axis++) {
    gyroRAW[axis] -= GYRO_Zero[axis];
    gyroRAW[axis] = constrain(gyroRAW[axis],lastGyro[axis]-800,lastGyro[axis]+800);  
    lastGyro[axis] = gyroRAW[axis];
  }
}

void gyro_calibrate()
{
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 400; j++)
    {
      gyro_getdata();

      if(gyroRAW[i] > gyroHigh[i]) gyroHigh[i] = gyroRAW[i];
      else if(gyroRAW[i] < gyroLow[i]) gyroLow[i] = gyroRAW[i];
    }
  }
  gyroCalibrated = 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


uint8_t mag_getdata() {
  static uint32_t t,tCal = 0;
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];
  uint8_t axis;
  if ( currentTime < t ) return 0; //each read is spaced by 100ms
  t = currentTime + 100000;
  
  Device_Mag_getADC();
    
  magRAW[ROLL]  = magRAW[ROLL]  * magScale[ROLL];
  magRAW[PITCH] = magRAW[PITCH] * magScale[PITCH];
  magRAW[YAW]   = magRAW[YAW]   * magScale[YAW];
    
  if (f.magCalibrated) {
    tCal = t;
    for(axis=0;axis<3;axis++) {
      MAG_Zero[axis] = 0;
      magZeroTempMin[axis] = magRAW[axis];
      magZeroTempMax[axis] = magRAW[axis];
    }
    f.magCalibrated = 0;
  }
  if (magInit) { // we apply offset only once mag calibration is done
    magRAW[ROLL]  -= MAG_Zero[ROLL];
    magRAW[PITCH] -= MAG_Zero[PITCH];
    magRAW[YAW]   -= MAG_Zero[YAW];
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
        MAG_Zero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])/2;
    }
  }
  return 1;
}
