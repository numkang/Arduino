void setup(){
  Serial.begin(115200);
  Serial1.begin(115200);
  //SerialOpen(115200);
  //SerialOpen1(115200);  
}

void loop(){
  
  n = Serial1.available();
  //n = SerialAvailable1();
  if (n > 0){
    c = Serial1.read();
    //c = SerialRead1();
    //Serial.println(c, BIN);
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

int i = 0;

  //PrintDebugPacketData();
  switch(UM6_Packet.Address){
    /*case UM6_QUAT_AB :
      Serial.print("UM6_QUAT_AB : ");
      if (UM6_Packet.HasData && !UM6_Packet.CommFail){
        i = (aPacketData[0] << 8) | aPacketData[1];
        DataA = i * QUAT_SCALE_FACTOR;
        i = (aPacketData[2] << 8) | aPacketData[3];
        DataB = i * QUAT_SCALE_FACTOR;
        if (UM6_Packet.DataLength > 4){
          i = (aPacketData[4] << 8) | aPacketData[5];
          DataC = i * QUAT_SCALE_FACTOR;
          i = (aPacketData[6] << 8) | aPacketData[7];
          DataD = i * QUAT_SCALE_FACTOR;
        }
      }
      PrintDebugFloatABCD(DataA,DataB,DataC,DataD);
      break;

    case UM6_ACCEL_PROC_XY :
      Serial.print("UM6_ACCEL_PROC_XY : ");
      if (UM6_Packet.HasData && !UM6_Packet.CommFail){
        i = (aPacketData[0] << 8) | aPacketData[1];
        DataA = i * ACCEL_SCALE_FACTOR;
        i = (aPacketData[2] << 8) | aPacketData[3];
        DataB = i * ACCEL_SCALE_FACTOR;
        if (UM6_Packet.DataLength > 4){
          i = (aPacketData[4] << 8) | aPacketData[5];
          DataC = i * ACCEL_SCALE_FACTOR;
        }
      }
      PrintDebugFloatABCD(DataA,DataB,DataC,DataD);
      break;

    case UM6_GYRO_PROC_XY :
      Serial.print("UM6_GYRO_PROC_XY : ");    
      if (UM6_Packet.HasData && !UM6_Packet.CommFail){
        i = (aPacketData[0] << 8) | aPacketData[1];
        DataA = i * GYRO_SCALE_FACTOR;
        i = (aPacketData[2] << 8) | aPacketData[3];
        DataB = i * GYRO_SCALE_FACTOR;
        if (UM6_Packet.DataLength > 4){
          i = (aPacketData[4] << 8) | aPacketData[5];
          DataC = i * GYRO_SCALE_FACTOR;
        }
      }
      PrintDebugFloatABCD(DataA,DataB,DataC,DataD);
      break;*/
      
      case UM6_EULER_PHI_THETA :
      if (UM6_Packet.HasData && !UM6_Packet.CommFail){
        i = (aPacketData[0] << 8) | aPacketData[1];
        DataA = i * 0.0109863;
        i = (aPacketData[2] << 8) | aPacketData[3];
        DataB = i * 0.0109863;
        if (UM6_Packet.DataLength > 4){
          i = (aPacketData[4] << 8) | aPacketData[5];
          DataC = i * 0.0109863;
        }
      }
      PrintDebugFloatABCD(DataA,DataB,DataC,DataD);
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

void PrintDebugPacketData(){
  Serial.print("N = ");
  Serial.print(n,DEC);
  Serial.print(" HD = ");
  Serial.print(UM6_Packet.HasData,BIN);
  Serial.print(" IB = ");
  Serial.print(UM6_Packet.IsBatch,BIN);
  Serial.print(" BL = ");
  Serial.print(UM6_Packet.BatchLength,DEC);
  Serial.print(" CF = ");
  Serial.print(UM6_Packet.CommFail,BIN);
  Serial.print(" AD = 0x");
  Serial.print(UM6_Packet.Address,HEX);
  Serial.print(" CS1 = 0x");
  Serial.print(UM6_Packet.Checksum1,HEX);
  Serial.print(" CS0 = 0x");
  Serial.print(UM6_Packet.Checksum0,HEX);
  Serial.print(" DL = ");
  Serial.print(UM6_Packet.DataLength,DEC);
  Serial.println(".");
}

void PrintDebugFloatABCD(float a, float b, float c, float d){
  Serial.print(" A = ");
  Serial.print(a,DEC);
  Serial.print(" B = ");
  Serial.print(b,DEC);
  Serial.print(" C = ");
  Serial.print(c,DEC);
  Serial.print(" D = ");
  Serial.print(d,DEC);
  Serial.println(".");
}
