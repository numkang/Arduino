void calibrateVector(){ //find Vector Matrix when vehicle is level
  m_project[X] = -(acal[Y]*(acal[X]*mcal[Y] - acal[Y]*mcal[X])) - (acal[Z]*(acal[X]*mcal[Z] - acal[Z]*mcal[X]));
  m_project[Y] = acal[X]  * (acal[X]*mcal[Y] - acal[Y]*mcal[X]) - acal[Z] * (acal[Y]*mcal[Z] - acal[Z]*mcal[Y]);
  m_project[Z] = acal[X]  * (acal[X]*mcal[Z] - acal[Z]*mcal[X]) + acal[Y] * (acal[Y]*mcal[Z] - acal[Z]*mcal[Y]);
  
  crossVector(acal, m_project, acalxmp);
  generate3x3(acal, m_project, acalxmp, acal_mp_acalxmp);
  inverse3x3 (acal_mp_acalxmp, acal_mp_acalxmp_inv);
  
  MultipleMatrix33x33(idealVec, acal_mp_acalxmp_inv, TransformMartix);
  MultipleMatrix33x31(TransformMartix, mcal, m_level);
  
  crossVector(idealAcc, m_level, idealAccxmlevel);
  generate3x3(idealAcc, m_level, idealAccxmlevel, idealAcc_mlevel_idealAccxmlevel);
  inverse3x3 (idealAcc_mlevel_idealAccxmlevel, idealAcc_mlevel_idealAccxmlevel_inv);
}

void TransformVector(){
  for(int i = 0; i < 3; i++){
    if(gyroRAW[i] >= gyroHigh[i] || gyroRAW[i] <= gyroLow[i])
    {
      dPS[i] = (float)gyroRAW[i] * 0.05;
      
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
  Gyro_AngZ = gyroAngle[2]; //YAW
  
  gyroVectorY = gyroVectorY*cos(Gyro_AngZ*DEG_TO_RAD)-gyroVectorX*sin(Gyro_AngZ*DEG_TO_RAD);
  gyroVectorX = gyroVectorY*sin(Gyro_AngZ*DEG_TO_RAD)+gyroVectorX*cos(Gyro_AngZ*DEG_TO_RAD);	

  gyroVectorZ = gyroVectorY*sin(Gyro_AngY*DEG_TO_RAD)+gyroVectorZ*cos(Gyro_AngY*DEG_TO_RAD);
  gyroVectorY = gyroVectorY*cos(Gyro_AngY*DEG_TO_RAD)-gyroVectorZ*sin(Gyro_AngY*DEG_TO_RAD);

  gyroVectorZ = gyroVectorX*sin(Gyro_AngX*DEG_TO_RAD)+gyroVectorZ*cos(Gyro_AngX*DEG_TO_RAD);
  gyroVectorX = gyroVectorX*cos(Gyro_AngX*DEG_TO_RAD)-gyroVectorZ*sin(Gyro_AngX*DEG_TO_RAD);
  
  trueVectorX = ComplementAlpha[0]*(gyroVectorY)+(1-ComplementAlpha[0])*(acal[0]); //apply complementary filter
  trueVectorY = ComplementAlpha[1]*(gyroVectorX)+(1-ComplementAlpha[1])*(acal[1]);
  trueVectorZ = ComplementAlpha[2]*(gyroVectorZ)+(1-ComplementAlpha[2])*(acal[2]);
  
  /*trueVectorR = sqrt(trueVectorX*trueVectorX + trueVectorY*trueVectorY + trueVectorZ*trueVectorZ);
  trueVectorX /= trueVectorR;
  trueVectorY /= trueVectorR;
  trueVectorZ /= trueVectorR;*/
  
  acal[0] = trueVectorX;
  acal[1] = trueVectorY;
  acal[2] = trueVectorZ;  
  
  m_project[X] = -(acal[Y]*(acal[X]*mcal[Y] - acal[Y]*mcal[X])) - acal[Z] * (acal[X]*mcal[Z] - acal[Z]*mcal[X]);
  m_project[Y] = acal[X]  * (acal[X]*mcal[Y] - acal[Y]*mcal[X]) - acal[Z] * (acal[Y]*mcal[Z] - acal[Z]*mcal[Y]);
  m_project[Z] = acal[X]  * (acal[X]*mcal[Z] - acal[Z]*mcal[X]) + acal[Y] * (acal[Y]*mcal[Z] - acal[Z]*mcal[Y]);
    
  crossVector(acal, m_project, acalxmp);
  generate3x3(acal, m_project, acalxmp, acal_mp_acalxmp);
  
  //crossVector(acal, mcal, acalxmcal);
  //generate3x3(acal, mcal, acalxmcal, acal_mcal_acalxmcal);
   
  //MultipleMatrix33x33(acal_mcal_acalxmcal, idealAcc_mlevel_idealAccxmlevel_inv, TransformMartix);  
  MultipleMatrix33x33(acal_mp_acalxmp, idealVec_inv, TransformMartix);  
  aizmelev_to_xyz();
  MultipleMatrix33x31(TransformMartix, xyz_init, xyz_new);
  xyz_to_azimelev();
}

void latlongalt_to_azimelev(){
  
}

void aizmelev_to_xyz(){
  xyz_init[X] = 30000  * cos(initAzim*PI/180) * cos(initElev*PI/180);
  xyz_init[Y] = 30000  * cos(initElev*PI/180) * sin(initAzim*PI/180);
  xyz_init[Z] = -30000 * sin(initElev*PI/180);
}

void xyz_to_azimelev(){
  newAzim = (atan2(xyz_new[Y], xyz_new[X]))*180/PI;
  newElev = (atan2(-xyz_new[Z] , sqrt(xyz_new[X]*xyz_new[X] + xyz_new[Y]*xyz_new[Y])))*180/PI;
}
