#if defined(CHR_UM6)

void CHR_UM6_getdata(){
  n = SerialAvailable1();
  if (n > 0){
    c = SerialRead1();
    switch(nState){
      case STATE_ZERO : // Begin. Look for 's'.
        // Start of new packet...
        Reset();
        if (c == 's'){
          nState = STATE_S;
        } else {
          nState = STATE_ZERO;
        }
        break;
      case STATE_S : // Have 's'. Look for 'n'.
        if (c == 'n'){
          nState = STATE_SN; 
        } else {
          nState = STATE_ZERO;
        }
        break;
      case STATE_SN : // Have 'sn'. Look for 'p'.
        if (c == 'p'){
          nState = STATE_SNP; 
        } else {
          nState = STATE_ZERO;
        }
        break;
      case STATE_SNP : // Have 'snp'. Read PacketType and calculate DataLength.
        UM6_Packet.HasData = 1 && (c & PT_HAS_DATA);
        UM6_Packet.IsBatch = 1 && (c & PT_IS_BATCH);
        UM6_Packet.BatchLength = ((c >> 2) & 0b00001111);
        UM6_Packet.CommFail = 1 && (c & PT_COMM_FAIL);
        nState = STATE_PT;
        if (UM6_Packet.IsBatch){
          UM6_Packet.DataLength = UM6_Packet.BatchLength * 4;
        } else {
          UM6_Packet.DataLength = 4;
        }
        break;
      case STATE_PT : // Have PacketType. Read Address.
        UM6_Packet.Address = c;
        nDataByteCount = 0;
        nState = STATE_READ_DATA; 
        break;
      case STATE_READ_DATA : // Read Data. (UM6_PT.BatchLength * 4) bytes.
        aPacketData[nDataByteCount] = c;
        nDataByteCount++;
        if (nDataByteCount >= UM6_Packet.DataLength){
          nState = STATE_CHK1;
        }
        break;
      case STATE_CHK1 : // Read Checksum 1
        UM6_Packet.Checksum1 = c;
        nState = STATE_CHK0;
        break;
      case STATE_CHK0 : // Read Checksum 0
        UM6_Packet.Checksum0 = c;
        nState = STATE_DONE;
        break;
      case STATE_DONE : // Entire packet consumed. Process packet
        ProcessPacket();
        nState = STATE_ZERO;
        break;
    }
  } 
}

void ProcessPacket(){
  int temp = 0;
  switch(UM6_Packet.Address){      
    case UM6_EULER_PHI_THETA :
      if (UM6_Packet.HasData && !UM6_Packet.CommFail){
        temp = (aPacketData[0] << 8) | aPacketData[1];
        euler[0] = temp * 0.0109863;
        temp = (aPacketData[2] << 8) | aPacketData[3];
        euler[1] = temp * 0.0109863;
        if (UM6_Packet.DataLength > 4){
          temp = (aPacketData[4] << 8) | aPacketData[5];
          heading = temp * 0.0109863;
        }
      }
      for(int i=0; i<3; i++) angle[i] = euler[i]*10;
    break;
    
    case UM6_GYRO_PROC_XY :
      if (UM6_Packet.HasData && !UM6_Packet.CommFail){
        temp = (aPacketData[0] << 8) | aPacketData[1];
        gyroData[0] = temp * 0.0610352;
        temp = (aPacketData[2] << 8) | aPacketData[3];
        gyroData[1] = temp * 0.0610352;
        if (UM6_Packet.DataLength > 4){
          temp = (aPacketData[4] << 8) | aPacketData[5];
          gyroData[2] = temp * 0.0610352;
        }
      }
    break;
    
    case UM6_ACCEL_PROC_XY :
      if (UM6_Packet.HasData && !UM6_Packet.CommFail){
        temp = (aPacketData[0] << 8) | aPacketData[1];
        accSmooth[0] = temp * 0.000183105 * 256;
        temp = (aPacketData[2] << 8) | aPacketData[3];
        accSmooth[1] = temp * 0.000183105 * 256;
        if (UM6_Packet.DataLength > 4){
          temp = (aPacketData[4] << 8) | aPacketData[5];
          accSmooth[2] = temp * 0.000183105 * 256;
        }
      }
    break;
    
    case UM6_MAG_PROC_XY :
      if (UM6_Packet.HasData && !UM6_Packet.CommFail){
        temp = (aPacketData[0] << 8) | aPacketData[1];
        magRAW[0] = temp * 0.000305176 * 256;
        temp = (aPacketData[2] << 8) | aPacketData[3];
        magRAW[1] = temp * 0.000305176 * 256;
        if (UM6_Packet.DataLength > 4){
          temp = (aPacketData[4] << 8) | aPacketData[5];
          magRAW[2] = temp * 0.000305176 * 256;
        }
      }
    break;
  }
}

void Reset(){
  UM6_Packet.HasData = false;
  UM6_Packet.IsBatch = false;
  UM6_Packet.BatchLength = 0;
  UM6_Packet.CommFail = false;
  UM6_Packet.Address = 0;
  UM6_Packet.Checksum1 = 0;
  UM6_Packet.Checksum0 = 0;
  UM6_Packet.DataLength = 0;
}

#endif
