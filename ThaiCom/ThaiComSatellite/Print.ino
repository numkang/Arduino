static uint32_t printTime;

void Print(uint32_t ms){  
  if(currentTime > printTime){
    printTime = currentTime + ms*1000;
    SerialPrint();
  }
}

void SerialPrint(){
 
  Serial.print(analogRead(potenPin_azim)); 
  Serial.print(" , "); 
  Serial.print(potenVal_azim);
  Serial.print("  ,  ");
  Serial.print(analogRead(potenPin_elev)); 
  Serial.print(" , "); 
  Serial.println(potenVal_elev);
       
  Serial.print(isense_data.ax);
  Serial.print("\t");
  Serial.print(isense_data.ay);
  Serial.print("\t");
  Serial.print(isense_data.az);
  Serial.print("\n");
  
  Serial.print("Vec : ");
  Serial.print(trueVectorX);
  Serial.print("\t");
  Serial.print(trueVectorY);
  Serial.print("\t");
  Serial.print(trueVectorZ);
  Serial.print("\n");
  
  /*Serial.print("gyroRAW : ");
  Serial.print(gyroAngle[X]); 
  Serial.print(" , "); 
  Serial.print(gyroAngle[Y]);
  Serial.print(" , ");
  Serial.println(gyroAngle[Z]);*/
  
  Serial.print("mcal : ");
  Serial.print(mcal[X]); 
  Serial.print(" , "); 
  Serial.print(mcal[Y]);
  Serial.print(" , ");
  Serial.println(mcal[Z]);
  Serial.print("acal : ");
  Serial.print(acal[X]); 
  Serial.print(" , "); 
  Serial.print(acal[Y]);
  Serial.print(" , ");
  Serial.println(acal[Z]);
  Serial.print("m_project : ");
  Serial.print(m_project[X]); 
  Serial.print(" , "); 
  Serial.print(m_project[Y]);
  Serial.print(" , ");
  Serial.println(m_project[Z]);
  /*Serial.print("m_level : ");
  Serial.print(m_level[X]); 
  Serial.print(" , "); 
  Serial.print(m_level[Y]);
  Serial.print(" , ");
  Serial.println(m_level[Z]);  
  Serial.print("no_inv : ");
  Serial.print(idealAcc_mlevel_idealAccxmlevel[0][0]);
  Serial.print("  ");
  Serial.print(idealAcc_mlevel_idealAccxmlevel[0][1]);
  Serial.print("  ");
  Serial.println(idealAcc_mlevel_idealAccxmlevel[0][2]);
  Serial.print(idealAcc_mlevel_idealAccxmlevel[1][0]);
  Serial.print("  ");
  Serial.print(idealAcc_mlevel_idealAccxmlevel[1][1]);
  Serial.print("  ");
  Serial.println(idealAcc_mlevel_idealAccxmlevel[1][2]);
  Serial.print(idealAcc_mlevel_idealAccxmlevel[2][0]);
  Serial.print("  ");
  Serial.print(idealAcc_mlevel_idealAccxmlevel[2][1]);
  Serial.print("  ");
  Serial.println(idealAcc_mlevel_idealAccxmlevel[2][2]);
  Serial.print("inv : ");
  Serial.print(idealAcc_mlevel_idealAccxmlevel_inv[0][0]);
  Serial.print("  ");
  Serial.print(idealAcc_mlevel_idealAccxmlevel_inv[0][1]);
  Serial.print("  ");
  Serial.println(idealAcc_mlevel_idealAccxmlevel_inv[0][2]);
  Serial.print(idealAcc_mlevel_idealAccxmlevel_inv[1][0]);
  Serial.print("  ");
  Serial.print(idealAcc_mlevel_idealAccxmlevel_inv[1][1]);
  Serial.print("  ");
  Serial.println(idealAcc_mlevel_idealAccxmlevel_inv[1][2]);
  Serial.print(idealAcc_mlevel_idealAccxmlevel_inv[2][0]);
  Serial.print("  ");
  Serial.print(idealAcc_mlevel_idealAccxmlevel_inv[2][1]);
  Serial.print("  ");
  Serial.println(idealAcc_mlevel_idealAccxmlevel_inv[2][2]);
  Serial.print("axm : ");
  Serial.print(acal_mcal_acalxmcal[0][0]);
  Serial.print("  ");
  Serial.print(acal_mcal_acalxmcal[0][1]);
  Serial.print("  ");
  Serial.println(acal_mcal_acalxmcal[0][2]);
  Serial.print(acal_mcal_acalxmcal[1][0]);
  Serial.print("  ");
  Serial.print(acal_mcal_acalxmcal[1][1]);
  Serial.print("  ");
  Serial.println(acal_mcal_acalxmcal[1][2]);
  Serial.print(acal_mcal_acalxmcal[2][0]);
  Serial.print("  ");
  Serial.print(acal_mcal_acalxmcal[2][1]);
  Serial.print("  ");
  Serial.println(acal_mcal_acalxmcal[2][2]);
  Serial.print("T : ");
  Serial.print(TransformMartix[0][0]);
  Serial.print("  ");
  Serial.print(TransformMartix[0][1]);
  Serial.print("  ");
  Serial.println(TransformMartix[0][2]);
  Serial.print(TransformMartix[1][0]);
  Serial.print("  ");
  Serial.print(TransformMartix[1][1]);
  Serial.print("  ");
  Serial.println(TransformMartix[1][2]);
  Serial.print(TransformMartix[2][0]);
  Serial.print("  ");
  Serial.print(TransformMartix[2][1]);
  Serial.print("  ");
  Serial.println(TransformMartix[2][2]);*/
  /*Serial.print("xyz : ");
  Serial.print(xyz_new[X]); 
  Serial.print(" , "); 
  Serial.print(xyz_new[Y]);
  Serial.print(" , ");
  Serial.println(xyz_new[Z]);*/
  Serial.print("ang : ");
  Serial.print(newAzim); 
  Serial.print(" , "); 
  Serial.println(newElev);
  Serial.println();
  
  /*for(int i=0; i<3; i++){
    Serial.print(nunchuck_buf[i]);
    Serial.print(" , ");
  }
  Serial.print(nunchuck_zbutton());
  Serial.print(" , ");
  Serial.print(nunchuck_cbutton());
  Serial.println();*/
  
  /*Serial.println(currentTime);
  Serial.println(previousTime);
  Serial.println(dt);
  Serial.println();*/
}
