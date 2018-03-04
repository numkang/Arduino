/*void headSerialResponse(uint8_t port, uint8_t err, uint8_t s)  {
  serialize8(port, 'i');
  serialize8(port, 'S');
  serialize8(port, 'C');
  serialize8(port, err ? '!' : '>');
  checksum[port] = 0; // start calculating a new checksum
  serialize8(port, s);
  serialize8(port, cmdISC[port]);
}

void headSerialReply(uint8_t port, uint8_t s) {
  headSerialResponse(port, 0, s);
}

void inline headSerialError(uint8_t port, uint8_t s) {
  headSerialResponse(port, 1, s);
}

void tailSerialReply(uint8_t port) {
  serialize8(port, checksum[port]);
  UartSendData(port);
}

void serialCom(uint8_t port) { //i S C < payload command value checksum
  uint8_t c;  
  static uint8_t offset[UART_NUMBER];
  static uint8_t dataSize[UART_NUMBER];
  static enum _serial_state {
    IDLE,
    HEADER_init,
    HEADER_S,
    HEADER_C,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state[UART_NUMBER]; // = IDLE;
  
  static enum _serial_state2 {
    IDLE2,
    HEADER_s,
    HEADER_p,
    HEADER_i,
    HEADER_d,
    HEADER_r,
    HEADER_l,
    HEADER_x,
    HEADER_10,
    HEADER_1,
    HEADER_point,
    HEADER_0_1,
    HEADER_0_01,
    HEADER_0_001,
    HEADER_0_0001
  } c_state2[UART_NUMBER];
  c_state2[port] = IDLE2;
  
  uint8_t cc = SerialAvailable(port);
  
  while(cc--){
    uint8_t bytesTXBuff = ((uint8_t)(serialHeadTX[port]-serialTailTX[port]))%TX_BUFFER_SIZE; // indicates the number of occupied bytes in TX buffer
    if (bytesTXBuff > TX_BUFFER_SIZE - 40 ) return; // ensure there is enough free TX buffer to go further (40 bytes margin)
     c = SerialRead(port);
     
    /*if (c_state[port] == IDLE) {
      c_state[port] = (c=='i') ? HEADER_init : IDLE; //Println(0,"a");
      if (c_state[port] == IDLE) evaluateOtherData(port,c); // evaluate all other incoming serial data
    } else if (c_state[port] == HEADER_init) {
      c_state[port] = (c=='S') ? HEADER_S : IDLE; //Println(0,"b");
    } else if (c_state[port] == HEADER_S) {
      c_state[port] = (c=='C') ? HEADER_C : IDLE; //Println(0,"c");
    } else if (c_state[port] == HEADER_C) {
      c_state[port] = (c=='<') ? HEADER_ARROW : IDLE;       //Println(0,"d");
    } else if (c_state[port] == HEADER_ARROW) {
      if (c > INBUF_SIZE) {  // now we are expecting the payload size
        c_state[port] = IDLE;
        continue;
      }
      cmdISC[port] = c;
      dataSize[port] = c;
      offset[port] = 0;
      checksum[port] = 0;
      indRX[port] = 0;
      checksum[port] ^= c;
      c_state[port] = HEADER_SIZE;  // the command is to follow      
    } else if (c_state[port] == HEADER_SIZE) {
      cmdISC[port] = c;
      checksum[port] ^= c;
      c_state[port] = HEADER_CMD;
    } else if (c_state[port] == HEADER_CMD && offset[port] < dataSize[port]) {
      checksum[port] ^= c;
      inBuf[offset[port]++][port] = c;
    } else if (c_state[port] == HEADER_CMD && offset[port] >= dataSize[port]) {
      if (checksum[port] == c) {  // compare calculated and transferred checksum
        evaluateCommand(port);  // we got a valid packet, evaluate it
      }
      //else if(c_state[port] == HEADER_CMD){
         //evaluateCommand(port);
      //}
      c_state[port] = IDLE;
    }*/    
    
/*    if (c_state2[port] == IDLE2) {    
      if(c == 's'){
        c_state2[port] = HEADER_s;
        Print(0,'s');
      }
      else if(c == 'x'){
        c_state2[port] = HEADER_x;
        Print(0,'x');
      }
      else if(c == 'p'){
        c_state2[port] = HEADER_p;
        Print(0,'p');
      }
      else if(c == 'i'){
        c_state2[port] = HEADER_i;
        Print(0,'i');
      }
      else if(c == 'd'){
        c_state2[port] = HEADER_d;
        Print(0,'d');
      }
      else if(c == 'r'){
        c_state2[port] = HEADER_r;
        Print(0,'r');
      }
      else if(c == 'l'){
        c_state2[port] = HEADER_l;
        Print(0,'l');
      }
      else{
        c_state2[port] = IDLE2;
      }
      kgain = c;
    Print(0," ");
  } 
  else if (c_state2[port] == HEADER_p || c_state2[port] == HEADER_i || c_state2[port] == HEADER_d || c_state2[port] == HEADER_r || c_state2[port] == HEADER_l) {
    gain = 10.0f*hex_c(c);    
    c_state2[port] = HEADER_10;
    //Print(0,c_state2[port]);
  } 
  else if (c_state2[port] == HEADER_10) {
    gain += 1.0f*hex_c(c);
    c_state2[port] = HEADER_1;
    //Print(0,c_state2[port]);
  }
  else if (c_state2[port] == HEADER_1) {
    c_state2[port] = HEADER_point;
    //Print(0,c_state2[port]);
  } 
  else if (c_state2[port] == HEADER_point) {
    gain += 0.1f*hex_c(c);
    c_state2[port] = HEADER_0_1;
    //Print(0,c_state2[port]);
  }  
  else if (c_state2[port] == HEADER_0_1) {
    gain += 0.01f*hex_c(c);
    c_state2[port] = HEADER_0_01;
    //Print(0,c_state2[port]);
  }
  else if (c_state2[port] == HEADER_0_01) {
    gain += 0.001f*hex_c(c);
    c_state2[port] = HEADER_0_001;
    //Print(0,c_state2[port]);
  }
  else if (c_state2[port] == HEADER_0_001) {
    gain += 0.0001f*hex_c(c);
    c_state2[port] = HEADER_0_0001;
    //Print(0,c_state2[port]);
  }
  else if (c_state2[port] == HEADER_0_0001 && c == ';') {
    c_state2[port] = IDLE2;
    Print(0,"gain: ");
    Println(0,gain);
    if(kgain=='p') KP = gain;
    else if(kgain=='i') KI = gain;
    else if(kgain=='d') KD = gain;
    else if(kgain=='r') setpoint = gain*(-1);
    else if(kgain=='l') setpoint = gain;
  }
  else if(c_state2[port] == HEADER_s) {
    f.c = c;
    if(f.c == '1'){ Println(0,"Start");
    }
    else{ Println(0,"Stop");
    }
    c_state2[port] = IDLE2;
  }
  else if(c_state2[port] == HEADER_x) {
    f.x = c;
    if(f.x == '1'){ Println(0,"Print");
    }
    else{ Println(0,"Stop Print");
    }
    c_state2[port] = IDLE2;
  }
    
  }
}

void evaluateCommand(uint8_t port) {
  switch(cmdISC[port]) {
    case ISC_BUFFERING:
      headSerialReply(port,3); 
      serialize8(port, VER1);
      serialize8(port, VER2);
      serialize8(port, VER3);      
    break;
     
    case ISC_KEEP_ALIVE:
      headSerialReply(port,1);
      serialize8(port, 1);
    break;
    
    case ISC_START_STOP:
      headSerialReply(port,0);
      f.c = read8(port);
    break;
    
    case ISC_IMU:
      headSerialReply(port,5);
      if(angle[ROLL] < 0) serialize8(port,1);
      else serialize8(port,0);
      int32_t angle_e7;
      angle_e7 = abs(angle[ROLL])*10000000;
      serialize32(port, angle_e7);
    break;
    
    case ISC_VDO_ANGLE:
      headSerialReply(port,0);
      vdo_angle_check = read8(port);
      int32_t vdo_angle_e7;
      vdo_angle_e7 = read32(port);
      if(vdo_angle_check == 1) vdo_angle_e7 *= -1;
      vdo_angle = vdo_angle_e7/10000000.0f;
      /*vdo_angle_temp = vdo_angle_e7/10000000.0f;
      vdo_angle = 0.9*vdo_angle_temp + (1-0.9)*vdo_angle_previous;
      vdo_angle_previous = vdo_angle;
      serialize8(port,vdo_angle);*/
   /* break;
    
    case ISC_VDO_USE:
      headSerialReply(port,0);
      f.vdo_active = read8(0);
      //serialize8(port,f.vdo_active);
    break;
    
    default:  // we do not know how to handle the (valid) message, indicate error ISC $M!
      headSerialError(port,0);
    break;
  }
  tailSerialReply(port);
}

// evaluate all other incoming serial data
void evaluateOtherData(uint8_t port, uint8_t sr) {
  switch (sr) {
  // Note: we may receive weird characters here which could trigger unwanted features during flight.
  //       this could lead to a crash easily.
  //       Please use if (!f.ARMED) where neccessary
  }
}

uint8_t hex_c(uint8_t n) {    // convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if(n>9)  n -= 7;
  n &= 0x0F;
  return n;
}*/
