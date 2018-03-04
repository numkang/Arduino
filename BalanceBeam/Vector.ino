void gyro_calibrate(){
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 400; j++)
    {
      gyro_getdata();

      if(gyroRAW[i] > gyroHigh[i]) gyroHigh[i] = gyroRAW[i];
      else if(gyroRAW[i] < gyroLow[i]) gyroLow[i] = gyroRAW[i];
    }
  }
}

void filter(){
  accR = sqrt(long(accRAW[0])*long(accRAW[0]) + long(accRAW[1])*long(accRAW[1]) + long(accRAW[2])*long(accRAW[2]));
  n_accX = accRAW[0]/accR*(-1);
  n_accY = accRAW[1]/accR*(-1);
  n_accZ = accRAW[2]/accR;
  
  for(int i = 0; i < 3; i++){
    if(gyroRAW[i] >= gyroHigh[i] || gyroRAW[i] <= gyroLow[i])
    {
      dPS[i] = (float)gyroRAW[i] * 0.00908; //this value was 0.0000085 but this gives better result
      
      if (currentTime > previousTime) dt = (float) (currentTime - previousTime)/1000000.0;
      else dt = (float) ((4294967295-previousTime)+currentTime)/1000000.0;
        
      //gyroAngle[i] += ((gyroLast[i] + dPS[i])/2) * dt;
      gyroAngle[i] = ((gyroLast[i] + dPS[i])/2) * dt; //average between two values
      
      if(gyroAngle[i] > 180) gyroAngle[i] -= 360;
      else if(gyroAngle[i] < -180) gyroAngle[i] += 360;
      
      gyroLast[i] = dPS[i];
    }
    else gyroLast[i] = 0;
  }
  
  Gyro_AngX = gyroAngle[0]*(-1); //PITCH
  Gyro_AngY = gyroAngle[1]; //ROLL
  Gyro_AngZ = gyroAngle[2]*(-1); //YAW
  
  /*gyroVectorY = gyroVectorY*cos(Gyro_AngZ)-gyroVectorX*sin(Gyro_AngZ); Full Rotational Matrix
    gyroVectorX = gyroVectorY*sin(Gyro_AngZ)+gyroVectorX*cos(Gyro_AngZ);	

    gyroVectorZ = gyroVectorY*sin(Gyro_AngY)+gyroVectorZ*cos(Gyro_AngY);
    gyroVectorY = gyroVectorY*cos(Gyro_AngY)-gyroVectorZ*sin(Gyro_AngY);

    gyroVectorZ = gyroVectorX*sin(Gyro_AngX)+gyroVectorZ*cos(Gyro_AngX);
    gyroVectorX = gyroVectorX*cos(Gyro_AngX)-gyroVectorZ*sin(Gyro_AngX);*/

  gyroVectorY = gyroVectorY-gyroVectorX*Gyro_AngZ*DEG_TO_RAD;
  gyroVectorX = gyroVectorY*Gyro_AngZ*DEG_TO_RAD+gyroVectorX;

  gyroVectorZ = gyroVectorY*Gyro_AngY*DEG_TO_RAD+gyroVectorZ;
  gyroVectorY = gyroVectorY-gyroVectorZ*Gyro_AngY*DEG_TO_RAD;

  gyroVectorZ = gyroVectorX*Gyro_AngX*DEG_TO_RAD+gyroVectorZ;
  gyroVectorX = gyroVectorX-gyroVectorZ*Gyro_AngX*DEG_TO_RAD;
  
  trueVectorX = ComplementAlpha*(gyroVectorY)+(1-ComplementAlpha)*(n_accX); //apply complementary filter
  trueVectorY = ComplementAlpha*(gyroVectorX)+(1-ComplementAlpha)*(n_accY);
  trueVectorZ = ComplementAlpha*(gyroVectorZ)+(1-ComplementAlpha)*(n_accZ);
  
  trueVectorR = sqrt(trueVectorX*trueVectorX + trueVectorY*trueVectorY + trueVectorZ*trueVectorZ);
  trueVectorX /= trueVectorR;
  trueVectorY /= trueVectorR;
  trueVectorZ /= trueVectorR;
  
  trueAng[0] = acos(abs(trueVectorX)/trueVectorR)*RAD_TO_DEG-90;
  if(trueVectorX < 0) { trueAng[0] = -trueAng[0]; }
  trueAng[0] *= (-1);

  trueAng[1] = acos(abs(trueVectorY)/trueVectorR)*RAD_TO_DEG-90.0;
  if(trueVectorY < 0) { trueAng[1] = -trueAng[1]; }
  trueAng[1] *= (-1);  
}
