//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                        Serial Communication Function                     //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

void evaluateCommand() {
  switch(cmdMSP) {
    case ISP_BUFFERING:
      headSerialReply(3); 
      serialize8(VER1);
      serialize8(VER2);
      serialize8(VER3);
    break;

    case ISP_RAW_IMU:
      headSerialReply(18);
      for(uint8_t i=0;i<3;i++) serialize16(accSmooth[i]);
      for(uint8_t i=0;i<3;i++) serialize16(gyroData[i]);
      for(uint8_t i=0;i<3;i++) serialize16(magRAW[i]);
    break;
     
    case ISP_KEEP_ALIVE:
      headSerialReply(1);
      serialize8(1);
    break;
  
    case ISP_ATTITUDE:
      headSerialReply(6);
      for(uint8_t i=0;i<2;i++) serialize16(angle[i]);
      serialize16(heading);
    break;
     
    case ISP_MOTORCMD:
      headSerialReply(12);
      for(uint8_t i=0;i<6;i++) serialize16(PWM_Command[i]);
    break;
     
    case ISP_RXDATA:
      headSerialReply(16);
      for(uint8_t i=0;i<8;i++) serialize16(rcData[i]);
    break;
     
    case ISP_GET_PID:
      headSerialReply(19);
      for(uint8_t axis=0;axis<2;axis++) serialize8(AKp[axis]);
      for(uint8_t axis=0;axis<2;axis++) serialize8(AKi[axis]);
      for(uint8_t axis=0;axis<2;axis++) serialize8(AKd[axis]);
      for(uint8_t axis=0;axis<3;axis++) serialize8(GKp[axis]);
      for(uint8_t axis=0;axis<3;axis++) serialize8(GKi[axis]);
      for(uint8_t axis=0;axis<3;axis++) serialize8(GKd[axis]);
      serialize8(AltKp);
      serialize8(AltKi);
      serialize8(AltKd);
      serialize8(HKp);
    break;
        
    case ISP_SET_PID:
      uint8_t temp_APGain[2],temp_AIGain[2],temp_ADGain[2];
      uint8_t temp_PGain[3],temp_IGain[3],temp_DGain[3];
      uint8_t temp_AltKp, temp_AltKi, temp_AltKd;
      uint8_t temp_HKp;
      for(uint8_t axis=0;axis<2;axis++){
        temp_APGain[axis] = read8();
        AKp[axis] = temp_APGain[axis];
        EEPROM.write(7+axis,AKp[axis]);
        delay(4);
      }
      for(uint8_t axis=0;axis<2;axis++){
        temp_AIGain[axis] = read8();
        AKi[axis] = temp_AIGain[axis];
        EEPROM.write(9+axis,AKi[axis]);
        delay(4);
      }
      for(uint8_t axis=0;axis<2;axis++){
        temp_ADGain[axis] = read8();
        AKd[axis] = temp_ADGain[axis];
        EEPROM.write(11+axis,AKd[axis]);
        delay(4);
      }
      for(uint8_t axis=0;axis<3;axis++){
        temp_PGain[axis] = read8();
        GKp[axis] = temp_PGain[axis];
        EEPROM.write(13+axis,GKp[axis]);
        delay(4);
      }
      for(uint8_t axis=0;axis<3;axis++){
        temp_IGain[axis] = read8();
        GKi[axis] = temp_IGain[axis];
        EEPROM.write(16+axis,GKi[axis]);
        delay(4);
      }
      for(uint8_t axis=0;axis<3;axis++){
        temp_DGain[axis] = read8();
        GKd[axis] = temp_DGain[axis];
        EEPROM.write(19+axis,GKd[axis]);
        delay(4);
      }
      temp_AltKp = read8();
      AltKp = temp_AltKp;
      EEPROM.write(22,AltKp);
      delay(4);
      temp_AltKi = read8();
      AltKi = temp_AltKi;
      EEPROM.write(23,AltKi);
      delay(4);
      temp_AltKd = read8();
      AltKd = temp_AltKd;
      EEPROM.write(24,AltKd);
      delay(4);
      temp_HKp = read8();
      HKp = temp_HKp;
      EEPROM.write(25,HKp);
      delay(4);
      headSerialReply(0);
    break;
        
    case ISP_ACC_CALIBRATION:
      #if !defined (CHR_UM6)
        calibratingA = 400;
      #endif
      headSerialReply(0);
    break;
    
    case ISP_GYRO_CALIBRATION:
      #if !defined (CHR_UM6)
        calibratingG = 400;
      #endif
      headSerialReply(0);
    break;
    
    case ISP_CPU_CYCLE:
      headSerialReply(2);
      serialize16(cycleTime);
    break;
    
    case ISP_MINMAX_CMD:
      headSerialReply(4);
      serialize16(MINCOMMAND);
      serialize16(MAXCOMMAND);
    break;
    
    case ISP_GET_ALT:
      #if defined(BAROMETER) && defined(useBARO)
        headSerialReply(4);
        serialize32(EstAlt);
      #endif
    break;
    
    case ISP_BARO_CALIBRATION:
      #if defined(BAROMETER) && defined(useBARO)
        calibratingB = 200;
        headSerialReply(0);
      #endif
    break;
    
    case ISP_RX_SEND_MOUSE:
      int16_t ch1_temp, ch2_temp;
      ch1_temp = read16();
      ch2_temp = read16();
      rcData[1] = ch1_temp;
      rcData[2] = ch2_temp;
      f.MOUSE_MODE = 1;
      headSerialReply(0);
    break;
    
    case ISP_MOUSE_MODE:
      f.MOUSE_MODE = 0;
      rcData[1] = rcValue[1];
      rcData[2] = rcValue[2];
      headSerialReply(0);
    break;
    
    case ISP_SET_CONFIG:
      uint8_t aircraft_temp, mode_temp;
      
      aircraft_temp = read8();
      mode_temp = read8();
      
      f.Aircraft_Config = aircraft_temp;      
      f.Mode_Config = mode_temp;
      
      EEPROM.write(46,f.Aircraft_Config);
      delay(4);
    break;
    
    case ISP_GET_CONFIG:
      headSerialReply(3);
      serialize8(f.Aircraft_Config);
      serialize8(f.Mode_Config);
      serialize8(sensor);
    break;    
    
    case ISP_KINECT_MODE:
      f.KINECT_MODE = !f.KINECT_MODE;
      //rcData[1] = rcValue[1];
      //rcData[2] = rcValue[2];
      headSerialReply(0);
    break;
    
    case ISP_RX_SEND_KINECT:
      int16_t ch1_tempp, ch2_tempp;
      ch1_tempp = read16();
      ch2_tempp = read16();
      rcData[1] = ch1_tempp;
      rcData[2] = ch2_tempp;
      f.KINECT_MODE = 1;
      headSerialReply(0);
    break;
    
    case ISP_HALL_SENSOR:
      headSerialReply(4);
      for(uint8_t i=0;i<2;i++) serialize16(HallValue[i]);
    break;
    
    default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
      headSerialError(0);
    break;
  }
  tailSerialReply();
}

// evaluate all other incoming serial data
void evaluateOtherData(uint8_t sr) {
  switch (sr) {
  // Note: we may receive weird characters here which could trigger unwanted features during flight.
  //       this could lead to a crash easily.
  //       Please use if (!f.ARMED) where neccessary
  }
}

void serialCom() { //$ M < payload command value checksum
  uint8_t c;  
  static uint8_t offset;
  static uint8_t dataSize;
  static enum _serial_state {
    IDLE,
    HEADER_START,
    HEADER_M,
    HEADER_ARROW,
    HEADER_SIZE,
    HEADER_CMD,
  } c_state = IDLE;
  
  while (SerialAvailable()) {
    uint8_t bytesTXBuff = ((uint8_t)(serialHeadTX-serialTailTX))%TX_BUFFER_SIZE; // indicates the number of occupied bytes in TX buffer
    if (bytesTXBuff > TX_BUFFER_SIZE - 40 ) return; // ensure there is enough free TX buffer to go further (40 bytes margin)
    c = SerialRead();

    if (c_state == IDLE) {
      c_state = (c=='$') ? HEADER_START : IDLE;
      if (c_state == IDLE) evaluateOtherData(c); // evaluate all other incoming serial data
    } else if (c_state == HEADER_START) {
      c_state = (c=='M') ? HEADER_M : IDLE;
    } else if (c_state == HEADER_M) {
      c_state = (c=='<') ? HEADER_ARROW : IDLE;
    } else if (c_state == HEADER_ARROW) {
      if (c > INBUF_SIZE) {  // now we are expecting the payload size
        c_state = IDLE;
        continue;
      }
      dataSize = c;
      offset = 0;
      checksum = 0;
      indRX = 0;
      checksum ^= c;
      c_state = HEADER_SIZE;  // the command is to follow
    } else if (c_state == HEADER_SIZE) {
      cmdMSP = c;
      checksum ^= c;
      c_state = HEADER_CMD;
    } else if (c_state == HEADER_CMD && offset < dataSize) {
      checksum ^= c;
      inBuf[offset++] = c;
    } else if (c_state == HEADER_CMD && offset >= dataSize) {
      if (checksum == c) {  // compare calculated and transferred checksum
        evaluateCommand();  // we got a valid packet, evaluate it
      }
      c_state = IDLE;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                     Serial Receive & Transmit Function                   //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

uint32_t read32() {
  uint32_t t = read16();
  t+= (uint32_t)read16()<<16;
  return t;
}
uint16_t read16() {
  uint16_t t = read8();
  t+= (uint16_t)read8()<<8;
  return t;
}
uint8_t read8()  {
  return inBuf[indRX++]&0xff;
}

void headSerialResponse(uint8_t err, uint8_t s)  {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  checksum = 0; // start calculating a new checksum
  serialize8(s);
  serialize8(cmdMSP);
}

void headSerialReply(uint8_t s) {
  headSerialResponse(0, s);
}

void inline headSerialError(uint8_t s) {
  headSerialResponse(1, s);
}

void tailSerialReply() {
  serialize8(checksum);
  UartSendData();
}

void serializeNames(PGM_P s) {
  for (PGM_P c = s; pgm_read_byte(c); c++) {
    serialize8(pgm_read_byte(c));
  }
}

void serialize32(uint32_t a) {
  serialize8((a    ) & 0xFF);
  serialize8((a>> 8) & 0xFF);
  serialize8((a>>16) & 0xFF);
  serialize8((a>>24) & 0xFF);
}

void serialize16(int16_t a) {
  serialize8((a   ) & 0xFF);
  serialize8((a>>8) & 0xFF);
}

void serialize8(uint8_t a) {
  uint8_t t = serialHeadTX;
  if (++t >= TX_BUFFER_SIZE) t = 0;
  serialBufferTX[t] = a;
  checksum ^= a;
  serialHeadTX = t;
}

void UartSendData() {
  UCSR0B |= (1<<UDRIE0);
}

void SerialWrite(uint8_t c){
  serialize8(c);
  UartSendData();
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                            Serial General Function                       //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

static void inline SerialOpen(uint32_t baud) {
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
  UCSR0A = (1<<U2X0); 
  UBRR0H = h; 
  UBRR0L = l; 
  UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
  
  #if defined(MEGA)
    UCSR1A = (1<<U2X1); 
    UBRR1H = h; 
    UBRR1L = l; 
    UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1);
  #endif
}

static void inline SerialEnd() {
  UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0));
  
  #if defined(MEGA)
    UCSR1B &= ~((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)|(1<<UDRIE1));
  #endif
}

uint8_t SerialAvailable() {
  return (serialHeadRX != serialTailRX);
}

uint8_t SerialAvailable1() {
  //return (serialHeadRX1 != serialTailRX1);
  return ((serialHeadRX1 - serialTailRX1))%DATA_BUFF_LEN;
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           Serial Interrupt Function                      //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#if defined(PROMINI) || defined(UNO) || defined(MEGA)
  #if defined(PROMINI) || defined(UNO)
  ISR(USART_UDRE_vect) {  // Serial 0 on a PROMINI
  #endif
  #if defined(MEGA)
  ISR(USART0_UDRE_vect) { // Serial 0 on a MEGA
  #endif
    uint8_t t = serialTailTX;
    if (serialHeadTX != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR0 = serialBufferTX[t];  // Transmit next byte in the ring
      serialTailTX = t;
    }
    if (t == serialHeadTX) UCSR0B &= ~(1<<UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
  }
#endif

#if defined(MEGA)
  ISR(USART1_UDRE_vect) { // Serial 1 on a MEGA
    uint8_t t1 = serialTailTX1;
    if (serialHeadTX1 != t1) {
      if (++t1 >= DATA_BUFF_LEN) t1 = 0;
      UDR1 = serialBufferTX1[t1];  // Transmit next byte in the ring
      serialTailTX1 = t1;
    }
    if (t1 == serialHeadTX1) UCSR1B &= ~(1<<UDRIE1);
  }
#endif

#if defined(PROMINI) || defined(UNO)
  ISR(USART_RX_vect)  { store_uart_in_buf(UDR0); }
#endif

#if defined(MEGA)
  ISR(USART0_RX_vect) { store_uart_in_buf(UDR0); }
  ISR(USART1_RX_vect) { store_uart_in_buf1(UDR1); }
#endif

uint8_t SerialRead() {
  uint8_t t = serialTailRX;
  uint8_t c = serialBufferRX[t];
  if (serialHeadRX!= t) {
    if (++t >= RX_BUFFER_SIZE) t = 0;
    serialTailRX = t;
  }
  return c;
}

static void inline store_uart_in_buf(uint8_t data) {
  uint8_t h = serialHeadRX;
  if (++h >= RX_BUFFER_SIZE) h = 0;
  if (h == serialTailRX) return; // we did not bite our own tail?
  serialBufferRX[serialHeadRX] = data;
  serialHeadRX = h;
}

uint8_t SerialRead1() {
  uint8_t t1 = serialTailRX1;
  uint8_t c1 = serialBufferRX1[t1];
  if (serialHeadRX1!= t1) {
    if (++t1 >= DATA_BUFF_LEN) t1 = 0;
    serialTailRX1 = t1;
  }
  return c1;
}

static void inline store_uart_in_buf1(uint8_t data) {
  uint8_t h1 = serialHeadRX1;
  if (++h1 >= DATA_BUFF_LEN) h1 = 0;
  if (h1 == serialTailRX1) return; // we did not bite our own tail?
  serialBufferRX1[serialHeadRX1] = data;
  serialHeadRX1 = h1;
}
