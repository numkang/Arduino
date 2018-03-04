
void gyro_init() {
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);             //PWR_MGMT_1    -- DEVICE_RESET 1
  delay(5);
  i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03);             //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
  i2c_writeReg(MPU6050_ADDRESS, 0x1A, MPU6050_DLPF_CFG); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
  i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);             //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
}

void gyro_getdata() {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x43);
  GYRO_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>2 , // range: +/- 8192; +/- 2000 deg/sec
                    ((rawADC[2]<<8) | rawADC[3])>>2 ,
                    ((rawADC[4]<<8) | rawADC[5])>>2 );
  gyro_remove_offset();
}

void acc_init() {
  i2c_writeReg(MPU6050_ADDRESS, 0x1C, 0x10);             //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
  //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
  //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480
}

void acc_getdata () {
  i2c_getSixRawADC(MPU6050_ADDRESS, 0x3B);
  ACC_ORIENTATION( ((rawADC[0]<<8) | rawADC[1])>>3 ,
                   ((rawADC[2]<<8) | rawADC[3])>>3 ,
                   ((rawADC[4]<<8) | rawADC[5])>>3 );
  acc_remove_offset();
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           remove offset function                         //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void acc_remove_offset(){
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
      ACC_Zero[ROLL]  = acc_sample[ROLL]/ACC_TO_SAMPLE;
      ACC_Zero[PITCH] = acc_sample[PITCH]/ACC_TO_SAMPLE;
      ACC_Zero[YAW]   = acc_sample[YAW]/ACC_TO_SAMPLE - ACC_1G;
      f.accCalibrated = 1;      
    }
    calibratingA--;
  }
  for (axis = 0; axis < 3; axis++) {
    //accRAW[axis] -= ACC_Zero[axis];
  }
}

void gyro_remove_offset() {
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
