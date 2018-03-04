void gyro_init(){
  i2c_writeReg(L3G4200D_Address, 0x20, 0b00001111);
  i2c_writeReg(L3G4200D_Address, 0x24, 0b00000000);
}

void gyro_getdata(){
  
  rawADC[1] = readI2Cdata(L3G4200D_Address, 0x29);
  rawADC[0] = readI2Cdata(L3G4200D_Address, 0x28);

  rawADC[3] = readI2Cdata(L3G4200D_Address, 0x2B);
  rawADC[2] = readI2Cdata(L3G4200D_Address, 0x2A);
  
  rawADC[5] = readI2Cdata(L3G4200D_Address, 0x2D);
  rawADC[4] = readI2Cdata(L3G4200D_Address, 0x2C);
  
  GYRO_ORIENTATION( ((rawADC[1]<<8) | rawADC[0]) ,
                    ((rawADC[3]<<8) | rawADC[2]) ,
                    ((rawADC[5]<<8) | rawADC[4]));
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


