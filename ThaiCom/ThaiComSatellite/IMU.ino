void init_adis16405(){
  // Start SPI Library and setup for ADIS 16405
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH); // disable device to start with
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV8); 
                      // Defulat on Mega is (16 / 4 Mhz
                      // ADIS 16405 should be <= 2 Mhz
  SPI.setDataMode(SPI_MODE3);
  
  // Initialize reset pin and set to 5v
  pinMode(reset_pin, OUTPUT);
  digitalWrite(reset_pin, HIGH); 
  SPI.begin();
  
  // Set message as initially unavailable
  isense_data.VALID_DATA = false;
}

void read_adis16405(struct imu_adis16405 *imuData_ptr ){
  // Useful Reference: http://ez.analog.com/message/54453#54453
  // also refer to UMN UAV Group code for interfacing with ADIS 16405
  
  // Send register address
  digitalWrite(chipSelectPin, LOW);
  // Burst Mode DIN sequence: 0x3E00
  SPI.transfer(0x3E);
  SPI.transfer(0x00);
  digitalWrite(chipSelectPin, HIGH);
 

  // UMN UAV Code would delay here to wait for IMU to respond, but
  // testing showed with or without delay things working fine.
  // delayMicroseconds(1); // If using delay, revisit the delay value
 

  // Initialize variables.  Burst mode, according to documentation, should
  // output 12 messages, starting with SUPPLY_OUT and ending with AUX_ADC.
  // However, testing (and UAV code) showed 13 messages are needed in order
  // that the last message's "ND flag" is zero, indicating no more data to be read.
  
  const int responseLength = 13*2;
  unsigned int tmp;
  int outputData[13];
  byte response[responseLength]={0};
  

  // Read Burst Mode Data
  // Note: for correct operation on the Mega, it was observed that the on/off of the
  //       chip select pin MUST occur inside the loop.  Doing otherwise causes intermittent
  //       data corruption.  The reason for this is partly unclear to me.
  for (int k = 0; k < responseLength; k++) {
    digitalWrite(chipSelectPin, LOW);
    response[k] = SPI.transfer(0x00);  // send a value of 0 to read the first byte returned:
    digitalWrite(chipSelectPin, HIGH); 
  }

  /* Debugging...*/
  /*for (int k = 0; k < responseLength; k++) {
    Serial.print(response[k], BIN);
    Serial.println(" ");
  }*/

  
  // All inertial sensor outputs are in 14-bit, twos complement format
  // Combine data bytes; each register covers two bytes, but uses only 14 bits.
  for ( int i = 0; i < responseLength; i += 2 ) {
    // From the two byte message, the upper byte arrived
    // first and then the lower byte.  Hence, we combine
    // the first 6 bits of the upper byte with the lower
    // byte into an unsigned int.
    tmp = (response[i] & 0x3F) * 256 + response[i+1];
    
    // Next, convert the 14 bit unsigned int into the
    // two's complement format and store as a (signed) int.
    // This if statement does exactly that.  
    if ( tmp > 0x1FFF ) {
      outputData[i/2] = (int)(-(0x4000 - tmp));
    }
      else {
      outputData[i/2] = (int)tmp; 
    }
  }
  
  
  	// set status flag if supply voltage is within 4.75 to 5.25
  	// this is a lame form of 'data validation' - testing showed cases 
  	// where this does fail to catch corrupt data.  But it is better than nothing!
	if ((outputData[0]*0.002418 > 4.25) && (outputData[0]*0.002418 < 5.25)){
	
	  // update imupacket
	  // *IMP* IMU axis alignment is different: Z=-Z_body, Y=-Y_body
	  imuData_ptr->Vs = outputData[0];   //unit: 2.418 mVolt
	  imuData_ptr->p  = outputData[1];   //unit: 0.05 deg/s
	  imuData_ptr->q  = outputData[2];
	  imuData_ptr->r  = outputData[3];
	  imuData_ptr->ax = outputData[4];   //unit: 3.33 mg
	  imuData_ptr->ay = outputData[5];
	  imuData_ptr->az = outputData[6];
	  imuData_ptr->mx = outputData[7];   //unit: 0.5 mgauss
	  imuData_ptr->my = outputData[8];
	  imuData_ptr->mz = outputData[9];
	  imuData_ptr->T  = outputData[10];  //unit: 0.14 C wrt 25 C
	  imuData_ptr->adc = outputData[11]; //unit: 806 microV

      imuData_ptr->VALID_DATA = true; // ready is ready to be printed 
      
        } else {
          imuData_ptr->VALID_DATA = false; // data not ready
        }
}

void IMUCalibrate(){
  //Magnetometer Vector
  mbx = isense_data.mx - (-11.6291);
  mby = isense_data.my - (36.4933);
  mbz = isense_data.mz - (43.6607);
  
  mcal[X] = mbx*invm1 + mby*invm2 + mbz*invm3;
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
  abx = isense_data.ax - (-8.1921);
  aby = isense_data.ay - (-1.8639);
  abz = isense_data.az - (-3.4441);
  
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
  
  gyroRAW[X] = isense_data.p;
  gyroRAW[Y] = isense_data.q;
  gyroRAW[Z] = isense_data.r;
}

void gyro_calibrate()
{
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 400; j++)
    {
      read_adis16405(&isense_data);
      gyroRAW[X] = isense_data.p;
      gyroRAW[Y] = isense_data.q;
      gyroRAW[Z] = isense_data.r;

      if(gyroRAW[i] > gyroHigh[i]) gyroHigh[i] = gyroRAW[i];
      else if(gyroRAW[i] < gyroLow[i]) gyroLow[i] = gyroRAW[i];
    }
  }
  gyroCalibrated = 1;
}
