void IMUCalibrate(){
  //Magnetometer Vector
  mbx = magRAW[ROLL] - (-11.6291);
  mby = magRAW[PITCH] - (36.4933);
  mbz = magRAW[YAW] - (43.6607);
  
  mcal[X] = mbx* invm1 + mby*invm2 + mbz*invm3;
  mcal[Y] = mbx*invm4 + mby*invm5 + mbz*invm6;
  mcal[Z] = mbx*invm7 + mby*invm8 + mbz*invm9;
  
  /*heading = atan2(mcal[X], mcal[Y]);
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
 
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
 
  // Convert radians to degrees for readability.
  headingDegrees = heading * 180/M_PI;*/
  
  //Acceleroter Vector
  abx = accRAW[ROLL] - (-8.1921);
  aby = accRAW[PITCH] - (-1.8639);
  abz = accRAW[YAW] - (-3.4441);
  
  acal[X] = abx*inva1 + aby*inva2 + abz*inva3;
  acal[Y] = abx*inva4 + aby*inva5 + abz*inva6;
  acal[Z] = abx*inva7 + aby*inva8 + abz*inva9;
  
  /*mcal[X] = isense_data.mx;
  mcal[Y] = isense_data.my;
  mcal[Z] = isense_data.mz;
  
  acal[X] = isense_data.ax;
  acal[Y] = isense_data.ay;
  acal[Z] = isense_data.az;
  
  unitVec(acal);
  unitVec(mcal);*/
}

//////////////////////////////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////////////////////////////////

void TransformVector(){  
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

void latlongalt_to_xyz(){
  xyz_init[X] = 6371000  * cos(initLat*PI/180) * cos(initLon*PI/180);
  xyz_init[Y] = 6371000  * cos(initLat*PI/180) * sin(initLon*PI/180);
  xyz_init[Z] = -6371000 * sin(initLat*PI/180);
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
