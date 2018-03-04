void headSerialResponse(uint8_t port, uint8_t err, uint8_t s)  {
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
  
  uint8_t cc = SerialAvailable(port);
  
  while(cc--){
    uint8_t bytesTXBuff = ((uint8_t)(serialHeadTX[port]-serialTailTX[port]))%TX_BUFFER_SIZE; // indicates the number of occupied bytes in TX buffer
    if (bytesTXBuff > TX_BUFFER_SIZE - 40 ) return; // ensure there is enough free TX buffer to go further (40 bytes margin)
     c = SerialRead(port);
     step_c = c;
     step_cc = 100;
    if (c_state[port] == IDLE) {
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
    }    
  }
}

void evaluateCommand(uint8_t port) {
  switch(cmdISC[port]) {    
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
}
