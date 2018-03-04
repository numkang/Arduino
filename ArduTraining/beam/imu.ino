void imu_init(){
  i2c_writeReg(MPU6050_ADDRESS, 107, 0b10000000);  //PWR_MGMT_1: DEVICE_RESET 1
  delay(5);
  i2c_writeReg(MPU6050_ADDRESS, 107, 0b00000011);  //PWR_MGMT_1: SLEEP 0, CYCLE 0, TEMP_DIS 0, CLKSEL 3
  i2c_writeReg(MPU6050_ADDRESS, 26,  0b00000010);  //CONFIG: SETUP GYRO&ACC 
  i2c_writeReg(MPU6050_ADDRESS, 27,  0b00000000);  //GYRO_CONFIG: FS_SEL = 0: Full scale 250 deg/sec
  i2c_writeReg(MPU6050_ADDRESS, 28,  0b00000000);  //ACC_CONFIG: FS_SEL = 0: Full scale 2g
}

void gyro_getdata(){
  
  byte gyro_x_h = readI2Cdata(MPU6050_ADDRESS, 67);
  byte gyro_x_l = readI2Cdata(MPU6050_ADDRESS, 68);
  gyroRAW[X] = (gyro_x_h<< 8) | gyro_x_l;
  gyroRAW[X] = (gyroRAW[X]/gyro_scale) - gyro_offset[X];

  byte gyro_y_h = readI2Cdata(MPU6050_ADDRESS, 69);
  byte gyro_y_l = readI2Cdata(MPU6050_ADDRESS, 70);
  gyroRAW[Y] = (gyro_y_h << 8) | gyro_y_l;
  gyroRAW[Y] = (gyroRAW[Y]/gyro_scale) - gyro_offset[Y];
  
  byte gyro_z_h = readI2Cdata(MPU6050_ADDRESS, 71);
  byte gyro_z_l = readI2Cdata(MPU6050_ADDRESS, 72);
  gyroRAW[Z] = (gyro_z_h << 8) | gyro_z_l;  
  gyroRAW[Z] = (gyroRAW[Z]/gyro_scale) - gyro_offset[Z];
}

void acc_getdata(){
  
  byte acc_x_h = readI2Cdata(MPU6050_ADDRESS, 59);
  byte acc_x_l = readI2Cdata(MPU6050_ADDRESS, 60);
  accRAW[X] = (acc_x_h << 8) | acc_x_l;
  accRAW[X] = (accRAW[X]/acc_scale) - acc_offset[X];

  byte acc_y_h = readI2Cdata(MPU6050_ADDRESS, 61);
  byte acc_y_l = readI2Cdata(MPU6050_ADDRESS, 62);
  accRAW[Y] = (acc_y_h << 8) | acc_y_l;
  accRAW[Y] = (accRAW[Y]/acc_scale) - acc_offset[Y];
  
  byte acc_z_h = readI2Cdata(MPU6050_ADDRESS, 63);
  byte acc_z_l = readI2Cdata(MPU6050_ADDRESS, 64);
  accRAW[Z] = (acc_z_h << 8) | acc_z_l; 
  accRAW[Z] = (accRAW[Z]/acc_scale) - acc_offset[Z] + 256;
}
