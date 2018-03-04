//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                     Serial Receive & Transmit Function                   //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

uint32_t read32(uint8_t port) {
  uint32_t t = read16(port);
  t+= (uint32_t)read16(port)<<16;
  return t;
}
uint16_t read16(uint8_t port) {
  uint16_t t = read8(port);
  t+= (uint16_t)read8(port)<<8;
  return t;
}
uint8_t read8(uint8_t port)  {
  return inBuf[indRX[port]++][port]&0xff;
}

void serialize32(uint8_t port, uint32_t a) {
  serialize8(port, (a    ) & 0xFF);
  serialize8(port, (a>> 8) & 0xFF);
  serialize8(port, (a>>16) & 0xFF);
  serialize8(port, (a>>24) & 0xFF);
}

void serialize16(uint8_t port, int16_t a) {
  serialize8(port, (a   ) & 0xFF);
  serialize8(port, (a>>8) & 0xFF);
}

void serialize8(uint8_t port, uint8_t a) {
  uint8_t t = serialHeadTX[port];
  if (++t >= TX_BUFFER_SIZE) t = 0;
  serialBufferTX[t][port] = a;
  checksum[port] ^= a;
  serialHeadTX[port] = t;
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                            Serial General Function                       //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void UartSendData(uint8_t port) {
  #if defined(PROMINI) || defined(UNO)
    UCSR0B |= (1<<UDRIE0);
  #endif
  #if defined(MEGA)
    switch (port) {
      case 0: UCSR0B |= (1<<UDRIE0); break;
      case 1: UCSR1B |= (1<<UDRIE1); break;
      case 2: UCSR2B |= (1<<UDRIE2); break;
      case 3: UCSR3B |= (1<<UDRIE3); break;
    }
  #endif
}

void SerialWrite(uint8_t port, uint8_t c){
  serialize8(port, c);
  UartSendData(port);
}

void inline SerialOpen(uint8_t port, uint32_t baud) {
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
  switch (port) {
    #if defined(PROMINI) || defined(UNO)
      case 0: UCSR0A  = (1<<U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); break;
    #endif
    #if defined(MEGA)
      case 0: UCSR0A  = (1<<U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); break;
      case 1: UCSR1A  = (1<<U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1); break;
      case 2: UCSR2A  = (1<<U2X2); UBRR2H = h; UBRR2L = l; UCSR2B |= (1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2); break;
      case 3: UCSR3A  = (1<<U2X3); UBRR3H = h; UBRR3L = l; UCSR3B |= (1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3); break;
    #endif
  }
}

void inline SerialEnd(uint8_t port) {
  switch (port) {
    #if defined(PROMINI)
      case 0: UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0)); break;
    #endif
    #if defined(MEGA)
      case 0: UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0)); break;
      case 1: UCSR1B &= ~((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)|(1<<UDRIE1)); break;
      case 2: UCSR2B &= ~((1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2)|(1<<UDRIE2)); break;
      case 3: UCSR3B &= ~((1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3)|(1<<UDRIE3)); break;
    #endif
  }
}

uint8_t SerialAvailable(uint8_t port) {
  return ((uint8_t)(serialHeadRX[port] - serialTailRX[port]))%RX_BUFFER_SIZE;
}

uint8_t SerialRead(uint8_t port) {
  uint8_t t = serialTailRX[port];
  uint8_t c = serialBufferRX[t][port];
  if (serialHeadRX[port] != t) {
    if (++t >= RX_BUFFER_SIZE) t = 0;
    serialTailRX[port] = t;
  }
  return c;
}

void inline store_uart_in_buf(uint8_t data, uint8_t portnum) {
  uint8_t h = serialHeadRX[portnum];
  if (++h >= RX_BUFFER_SIZE) h = 0;
  if (h == serialTailRX[portnum]) return; // we did not bite our own tail?
  serialBufferRX[serialHeadRX[portnum]][portnum] = data;  
  serialHeadRX[portnum] = h;
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           Serial Interrupt Function                      //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#if defined(PROMINI) || defined(MEGA) || defined(UNO)
  #if defined(PROMINI)
  ISR(USART_UDRE_vect) {  // Serial 0 on a PROMINI
  #endif
  #if defined(MEGA)
  ISR(USART0_UDRE_vect) { // Serial 0 on a MEGA
  #endif
    uint8_t t = serialTailTX[0];
    if (serialHeadTX[0] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR0 = serialBufferTX[t][0];  // Transmit next byte in the ring
      serialTailTX[0] = t;
    }
    if (t == serialHeadTX[0]) UCSR0B &= ~(1<<UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
  }
#endif
#if defined(MEGA)
  ISR(USART1_UDRE_vect) { // Serial 1 on a MEGA or on a PROMICRO
    uint8_t t = serialTailTX[1];
    if (serialHeadTX[1] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR1 = serialBufferTX[t][1];  // Transmit next byte in the ring
      serialTailTX[1] = t;
    }
    if (t == serialHeadTX[1]) UCSR1B &= ~(1<<UDRIE1);
  }
  
  ISR(USART2_UDRE_vect) { // Serial 2 on a MEGA
    uint8_t t = serialTailTX[2];
    if (serialHeadTX[2] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR2 = serialBufferTX[t][2];
      serialTailTX[2] = t;
    }
    if (t == serialHeadTX[2]) UCSR2B &= ~(1<<UDRIE2);
  }
  
  ISR(USART3_UDRE_vect) { // Serial 3 on a MEGA
    uint8_t t = serialTailTX[3];
    if (serialHeadTX[3] != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR3 = serialBufferTX[t][3];
      serialTailTX[3] = t;
    }
    if (t == serialHeadTX[3]) UCSR3B &= ~(1<<UDRIE3);
  }
#endif

#if defined(PROMINI)
  ISR(USART_RX_vect)  { store_uart_in_buf(UDR0, 0); }
#endif

#if defined(MEGA)
  ISR(USART0_RX_vect)  { store_uart_in_buf(UDR0, 0); }
  ISR(USART1_RX_vect)  { store_uart_in_buf(UDR1, 1); }
  ISR(USART2_RX_vect)  { store_uart_in_buf(UDR2, 2); }
  ISR(USART3_RX_vect)  { store_uart_in_buf(UDR3, 3); }
#endif

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                             Serial Print Function                        //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void Print(uint8_t port, uint8_t a){
  String c = (String)(a);
   for(uint8_t i=0; i<c.length(); i++){
     SerialWrite(port, c[i]);
   }  
}
void Print(uint8_t port, int16_t a){
  String c = (String)(a);
   for(uint8_t i=0; i<c.length(); i++){
     SerialWrite(port, c[i]);
   }  
}

void Print(uint8_t port, uint16_t a){
  String c = (String)(a);
   for(uint8_t i=0; i<c.length(); i++){
     SerialWrite(port, c[i]);
   }  
}

void Print(uint8_t port, int32_t a){
  String c = (String)(a);
   for(uint8_t i=0; i<c.length(); i++){
     SerialWrite(port, c[i]);
   }  
}

void Print(uint8_t port, uint32_t a){
  String c = (String)(a);
   for(uint8_t i=0; i<c.length(); i++){
     SerialWrite(port, c[i]);
   }  
}

void Print(uint8_t port, float a){
  if (a > 4294967040.0) Print(port, "ovf");
  else if (a <-4294967040.0) Print (port, "ovf");
  else{
    int32_t c;
    Print(port, (int32_t)a);
    Print(port, '.');
    for(uint8_t i=0; i<7; i++){
      a*=10;
      c = (int32_t)a;
      Print(port, abs(c%10));
    } 
  }
}

void Print(uint8_t port, double a){
  if (a > 4294967040.0) Print(port, "ovf");
  else if (a <-4294967040.0) Print (port, "ovf");
  else{
    int32_t c;
    Print(port, (int32_t)a);
    Print(port, '.');
    for(uint8_t i=0; i<7; i++){
      a*=10;
      c = (int32_t)a;
      Print(port, abs(c%10));
    } 
  }
}

void Print(uint8_t port, char c){
  SerialWrite(port, c);
}

void Print(uint8_t port, String c){
   for(uint8_t i=0; i<c.length(); i++){
     SerialWrite(port, c[i]);
   }  
}

///////////////////////////////////////////////////////////////////////

void Println(uint8_t port){
   SerialWrite(port, 10); 
}

void Println(uint8_t port, uint8_t c){
   Print(port, c);
   SerialWrite(port, 10); 
}

void Println(uint8_t port, int16_t c){
   Print(port, c);
   SerialWrite(port, 10); 
}

void Println(uint8_t port, uint16_t c){
   Print(port, c);
   SerialWrite(port, 10); 
}

void Println(uint8_t port, int32_t c){
   Print(port, c);
   SerialWrite(port, 10); 
}

void Println(uint8_t port, uint32_t c){
   Print(port, c);
   SerialWrite(port, 10); 
}

void Println(uint8_t port, float c){
   Print(port, c);
   SerialWrite(port, 10); 
}

void Println(uint8_t port, double c){
   Print(port, c);
   SerialWrite(port, 10); 
}

void Println(uint8_t port, char c){
   Print(port, c);
   SerialWrite(port, 10); 
}

void Println(uint8_t port, String c){
   Print(port, c);
   SerialWrite(port, 10); 
}

///////////////////////////////////////////////////////////////////////

void PrintTab(uint8_t port){
  SerialWrite(port, 9);  
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                  Protocol                                //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

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

void evaluateCommand(uint8_t port) {
  switch(cmdISC[port]) {     
    case ISC_KEEP_ALIVE:
      headSerialReply(port,1);
      serialize8(port, isAlive++);
    break;

    case ISC_ANGLE:
      headSerialReply(port,2);
      serialize16(port, angle[PITCH]+900);
    break;

    case ISC_SETPOINT:
      headSerialReply(port,0);
      setpoint = read8(port);
      digitalWrite(2, HIGH);
      delay(100);
      digitalWrite(2, LOW);
    break;

    case ISC_GAIN:
      headSerialReply(port,0);
      uint8_t temp_Gain[6];
      for(uint8_t i=0; i<6; i++){
        temp_Gain[i] = read8(port);
        if(i==0) K[P] = temp_Gain[i];
        else if(i==1) K[P] += temp_Gain[i]/100.0;
        else if(i==2) K[I] = temp_Gain[i];
        else if(i==3) K[I] += temp_Gain[i]/100.0;
        else if(i==4) K[D] = temp_Gain[i];
        else if(i==5) K[D] += temp_Gain[i]/100.0;
      }
      digitalWrite(2, HIGH);
      delay(100);
      digitalWrite(2, LOW);
    break;

    case ISC_ON:
      headSerialReply(port,0);      
      isConnect = true;
      digitalWrite(13,HIGH);
    break;

    case ISC_OFF:
      headSerialReply(port,0);
      isConnect = false;
      digitalWrite(13,LOW);
    break;

    case ISC_START:
      headSerialReply(port,0);
      errorI = 0;
      errorD = 0;
      previous_error = 0;
      f.ARM = 1;
    break;

    case ISC_STOP:
      headSerialReply(port,0);
      f.ARM = 0;      
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

void serialCom(uint8_t port) { //i S C < payload command value checksum (from UI)
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
      c_state[port] = IDLE;
    }
  }
}

uint8_t hex_c(uint8_t n) {    // convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if(n>9)  n -= 7;
  n &= 0x0F;
  return n;
}
