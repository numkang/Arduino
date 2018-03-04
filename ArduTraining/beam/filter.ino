void filter_gyro(){      
  for(int i=0; i<3; i++){
    if (currentTime > previousTime) dt = (float) (currentTime - previousTime)/1000000.0;
    else dt = (float) ((4294967295-previousTime)+currentTime)/1000000.0;
      
    //gyroAngle[i] = gyroRAW[i] * dt;  
    gyroAngle[i] = gyroRAW[i] * dt; 
    //gyroAngle[i] += ((gyroLast[i] + gyroRAW[i])/2) * dt;
    //gyroAngle[i] = ((gyroLast[i] + gyroRAW[i])/2) * dt; //average between two values
      
    if(gyroAngle[i] > 180) gyroAngle[i] -= 360;
    else if(gyroAngle[i] < -180) gyroAngle[i] += 360;
      
    //gyroLast[i] = gyroRAW[i];
  }
}

void filter_acc(){  
  x_val = (float)accRAW[X]*(-1);
  y_val = (float)accRAW[Y];
  z_val = (float)accRAW[Z];
   
  x2 = (unsigned long)(long(x_val)*long(x_val));
  y2 = (unsigned long)(long(y_val)*long(y_val));
  z2 = (unsigned long)(long(z_val)*long(z_val));

  acc_result = sqrt(x2+z2);
  acc_result = y_val/acc_result;
  accAngle[ROLL] = atan(acc_result)*RAD_TO_DEG;

  acc_result = sqrt(y2+z2);
  acc_result = x_val/acc_result;
  accAngle[PITCH] = atan(acc_result)*RAD_TO_DEG;
}

void com_filter(){
  acc_result = sqrt(x2+y2+z2);
  n_accX = x_val/acc_result;
  n_accY = y_val/acc_result;
  n_accZ = z_val/acc_result;
  
  Gyro_AngX = gyroAngle[0]*(-1);
  Gyro_AngY = gyroAngle[1]*(-1);
  Gyro_AngZ = gyroAngle[2]*(-1);
  
  gyroVectorY = gyroVectorY-gyroVectorX*Gyro_AngZ*DEG_TO_RAD;
  gyroVectorX = gyroVectorY*Gyro_AngZ*DEG_TO_RAD+gyroVectorX;

  gyroVectorZ = gyroVectorY*Gyro_AngY*DEG_TO_RAD+gyroVectorZ;
  gyroVectorY = gyroVectorY-gyroVectorZ*Gyro_AngY*DEG_TO_RAD;

  gyroVectorZ = gyroVectorX*Gyro_AngX*DEG_TO_RAD+gyroVectorZ;
  gyroVectorX = gyroVectorX-gyroVectorZ*Gyro_AngX*DEG_TO_RAD;
  
  trueVectorX = ComplementAlpha*(gyroVectorX)+(1-ComplementAlpha)*(n_accY); //apply complementary filter
  trueVectorY = ComplementAlpha*(gyroVectorY)+(1-ComplementAlpha)*(n_accX);
  trueVectorZ = ComplementAlpha*(gyroVectorZ)+(1-ComplementAlpha)*(n_accZ);
  
  trueVectorR = sqrt(trueVectorX*trueVectorX + trueVectorY*trueVectorY + trueVectorZ*trueVectorZ);
  
  trueAngle[ROLL] = acos(abs(trueVectorX)/trueVectorR)*RAD_TO_DEG-90;
  if(trueVectorX > 0) { trueAngle[X] = -trueAngle[X]; }

  trueAngle[PITCH] = acos(abs(trueVectorY)/trueVectorR)*RAD_TO_DEG-90;
  if(trueVectorY > 0) { trueAngle[Y] = -trueAngle[Y]; }
}
