
void GYRO_Common() {
  static int16_t previousGyroADC[3] = {0,0,0};
  static int32_t g[3];
  uint8_t axis;
  

  if (calibratingG>0) {
    for (axis = 0; axis < 3; axis++) {
      if (calibratingG == GYRO_SAMPLES_TO_CALIBRATE) g[axis]=0;
      g[axis] +=gyroADC[axis];
      gyroADC[axis]=0;
      gyroZero[axis]=0;
      if (calibratingG == 1) {
        gyroZero[axis]=g[axis]/GYRO_SAMPLES_TO_CALIBRATE;  
        f.gyroCalibrated = 1;      
      }
    }
    calibratingG--;
  }

  for (axis = 0; axis < 3; axis++) {
    gyroADC[axis]  -= gyroZero[axis];
    gyroADC[axis] = constrain(gyroADC[axis],previousGyroADC[axis]-800,previousGyroADC[axis]+800);
    previousGyroADC[axis] = gyroADC[axis];
  }
}

void ACC_Common() {
  static int32_t a[3];
  
  if (calibratingA>0) {
    for (uint8_t axis = 0; axis < 3; axis++) {
      if (calibratingA == ACC_SAMPLES_TO_CALIBRATE) a[axis]=0;
      a[axis] +=accADC[axis];
      accADC[axis]=0;
      conf.accZero[axis]=0;
    }
    if (calibratingA == 1) {
      conf.accZero[ROLL]  = a[ROLL]/ACC_SAMPLES_TO_CALIBRATE;
      conf.accZero[PITCH] = a[PITCH]/ACC_SAMPLES_TO_CALIBRATE;
      conf.accZero[YAW]   = a[YAW]/ACC_SAMPLES_TO_CALIBRATE -acc_1G; // for nunchuk 200=1G
    f.accCalibrated = 1;
  }
    calibratingA--;
  }
  accADC[ROLL]  -=  conf.accZero[ROLL] ;
  accADC[PITCH] -=  conf.accZero[PITCH];
  accADC[YAW]   -=  conf.accZero[YAW] ;
}


void Gyro_init() {
  delay(100);
  i2c_writeReg(L3G4200D_ADDRESS ,0x20 ,0x8F ); // CTRL_REG1   400Hz ODR, 20hz filter, run!
  delay(5);
  i2c_writeReg(L3G4200D_ADDRESS ,0x24 ,0x02 ); // CTRL_REG5   low pass filter enable
  delay(10);
}

void Gyro_getADC () {
  //TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  i2c_getSixRawADC(L3G4200D_ADDRESS,0x80|0x28);

  GYRO_ORIENTATION( ((rawADC[1]<<8) | rawADC[0])/20  ,
                    ((rawADC[3]<<8) | rawADC[2])/20  ,
                    ((rawADC[5]<<8) | rawADC[4])/20  );
  GYRO_Common();
}

void ACC_init () {
  delay(10);
  i2c_writeReg(ADXL345_ADDRESS,0x2D,1<<3);
  delay(10); //  register: Power CTRL  -- value: Set measure bit 3 on
  i2c_writeReg(ADXL345_ADDRESS,0x31,0x0B);
  delay(10); //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
  i2c_writeReg(ADXL345_ADDRESS,0x2C,0x09);
  delay(10); //  register: BW_RATE     -- value: rate=50hz, bw=20hz
  acc_1G = 265;
}

void ACC_getADC () {
  i2c_getSixRawADC(ADXL345_ADDRESS,0x32);

  ACC_ORIENTATION( ((rawADC[1]<<8) | rawADC[0]) ,
                   ((rawADC[3]<<8) | rawADC[2]) ,
                   ((rawADC[5]<<8) | rawADC[4]) );
  ACC_Common();
}

void initSensors() {
i2c_init();
delay(100);
Gyro_init();
delay(10);
Mag_init();
delay(10);
ACC_init();
acc_25deg = acc_1G * 0.423;
delay(10);
}

void Mag_init() { 
    delay(100);    // force positiveBias
    i2c_writeReg(MAG_ADDRESS ,0x00 ,0x71 ); //Configuration Register A  -- 0 11 100 01  num samples: 8 ; output rate: 15Hz ; positive bias
    delay(50);
    // set gains for calibration
    i2c_writeReg(MAG_ADDRESS ,0x01 ,0x60 ); //Configuration Register B  -- 011 00000    configuration gain 2.5Ga
    i2c_writeReg(MAG_ADDRESS ,0x02 ,0x01 ); //Mode register             -- 000000 01    single Conversion Mode
    // read values from the compass -  self test operation
    // by placing the mode register into single-measurement mode (0x01), two data acquisition cycles will be made on each magnetic vector.
    // The first acquisition values will be subtracted from the second acquisition, and the net measurement will be placed into the data output registers
    delay(100);
      getADC();
    delay(10);
      magCal[ROLL]  =  1160.0 / abs(magADC[ROLL]);
      magCal[PITCH] =  1160.0 / abs(magADC[PITCH]);
      magCal[YAW]   =  1080.0 / abs(magADC[YAW]);


    // leave test mode
    i2c_writeReg(MAG_ADDRESS ,0x00 ,0x70 ); //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    i2c_writeReg(MAG_ADDRESS ,0x01 ,0x20 ); //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    i2c_writeReg(MAG_ADDRESS ,0x02 ,0x00 ); //Mode register             -- 000000 00    continuous Conversion Mode

    magInit = 1;
  }

void getADC() {
  i2c_getSixRawADC(MAG_ADDRESS,MAG_DATA_REGISTER);
    MAG_ORIENTATION( ((rawADC[0]<<8) | rawADC[1]) ,
                     ((rawADC[4]<<8) | rawADC[5]) ,
                     ((rawADC[2]<<8) | rawADC[3]) );
}


void Device_Mag_getADC() {
  getADC();
}



void Mag_getADC() {
  static uint32_t t,tCal = 0;
  static int16_t magZeroTempMin[3];
  static int16_t magZeroTempMax[3];
  uint8_t axis;
  if ( currentTime < t ) return; //each read is spaced by 100ms
  t = currentTime + 100000;
  TWBR = ((F_CPU / 400000L) - 16) / 2; // change the I2C clock rate to 400kHz
  Device_Mag_getADC();
  magADC[ROLL]  = magADC[ROLL]  * magCal[ROLL];
  magADC[PITCH] = magADC[PITCH] * magCal[PITCH];
  magADC[YAW]   = magADC[YAW]   * magCal[YAW];
  if (f.CALIBRATE_MAG) {
    tCal = t;
    for(axis=0;axis<3;axis++) {
      conf.magZero[axis] = 0;
      magZeroTempMin[axis] = magADC[axis];
      magZeroTempMax[axis] = magADC[axis];
    }
    f.CALIBRATE_MAG = 0;
  }
  if (magInit) { // we apply offset only once mag calibration is done
    magADC[ROLL]  -= conf.magZero[ROLL];
    magADC[PITCH] -= conf.magZero[PITCH];
    magADC[YAW]   -= conf.magZero[YAW];
  }
 
  if (tCal != 0) {
    if ((t - tCal) < 30000000) { // 30s: you have 30s to turn the multi in all directions
      for(axis=0;axis<3;axis++) {
        if (magADC[axis] < magZeroTempMin[axis]) magZeroTempMin[axis] = magADC[axis];
        if (magADC[axis] > magZeroTempMax[axis]) magZeroTempMax[axis] = magADC[axis];
      }
    } else {
      tCal = 0;
      for(axis=0;axis<3;axis++)
        conf.magZero[axis] = (magZeroTempMin[axis] + magZeroTempMax[axis])/2;
    }
  }
}

