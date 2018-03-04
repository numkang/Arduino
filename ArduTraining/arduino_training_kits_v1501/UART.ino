#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define UART_NUMBER 4
#else
  #define UART_NUMBER 1
#endif
#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 128
#define INBUF_SIZE 64

static volatile uint8_t serialHeadRX[UART_NUMBER],serialTailRX[UART_NUMBER];
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t headTX,tailTX;
static uint8_t bufTX[TX_BUFFER_SIZE];
static uint8_t inBuf[INBUF_SIZE];



#define ISC_BUFFERING            100
#define ISC_RAW_IMU              102   //out message         9 DOF
#define ISC_ATTITUDE             108   //out message         2 angles 1 heading

#define ISC_MOTORS_PWM           120
#define ISC_SET_MOTOR_SPD        121
#define ISC_SET_PID              122
#define ISC_GET_PID              123
#define ISC_GYRO_CALIBRATION     124

#define ISC_CTRL_MODE            134

#define ISC_ACC_CALIBRATION      205  
#define ISC_MAG_CALIBRATION      206   



static uint8_t checksum;
static uint8_t indRX;
static uint8_t cmdMSP;

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

void headSerialResponse(uint8_t err, uint8_t s) {
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
  serialize8(checksum);UartSendData();
}

/*void serializeNames(PGM_P s) {
  for (PGM_P c = s; pgm_read_byte(c); c++) {
    serialize8(pgm_read_byte(c));
  }
}*/

void serialCom() {
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
  
  while (SerialAvailable(0)) {
    uint8_t bytesTXBuff = ((uint8_t)(headTX-tailTX))%TX_BUFFER_SIZE; // indicates the number of occupied bytes in TX buffer
    if (bytesTXBuff > TX_BUFFER_SIZE - 40 ) return; // ensure there is enough free TX buffer to go further (40 bytes margin)
    c = SerialRead(0);

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

void evaluateCommand() {
  switch(cmdMSP) {
   case ISC_CTRL_MODE:
   f.CTRL_MODE = read8();
   break;
   
   case ISC_SET_PID:
   uint16_t pg,ig,dg;
   pg = read16();
   ig = read16();
   dg = read16();
   PIDgains[P] = pg;
   PIDgains[I] = ig;
   PIDgains[D] = dg;
 //  PIDgains[P] = read16();
//   PIDgains[I] = read16();
 //  PIDgains[D] = read16();
   break;
   
   case ISC_GET_PID:
   headSerialReply(6);
   serialize16(PIDgains[P]);
   serialize16(PIDgains[I]);
   serialize16(PIDgains[D]);
   break;

  case ISC_BUFFERING:
  headSerialReply(3); 
  serialize8(VER);
  serialize8(VER);
  serialize8(VER);
  break;

   case ISC_RAW_IMU:
     headSerialReply(18);
     for(uint8_t i=0;i<3;i++) serialize16(accSmooth[i]);
     for(uint8_t i=0;i<3;i++) serialize16(gyroData[i]);
     for(uint8_t i=0;i<3;i++) serialize16(magADC[i]);
     break;
     
     case ISC_MOTORS_PWM:
     headSerialReply(4);
     uint16_t send_m1,send_m2;
     send_m1 = pPWM[M1];
     send_m2 = pPWM[M2];
    send_m1 = constrain(send_m1,1000,2000);
    send_m2 = constrain(send_m2,1000,2000);
     serialize16(send_m1);
     serialize16(send_m2);
    // serialize16(pPWM[M1]);
    // serialize16(pPWM[M2]);
     break;
     
     case ISC_SET_MOTOR_SPD:
     motorCMD[M1] = read16();
     motorCMD[M2] = read16();
     break;
  
   case ISC_ATTITUDE:
     headSerialReply(6);
     for(uint8_t i=0;i<2;i++) serialize16(angle[i]);
     serialize16(heading);
     break;
           
       case ISC_ACC_CALIBRATION:
     calibratingA=400;     
     ACC_SAMPLES_TO_CALIBRATE = calibratingA;
     headSerialReply(0);
     break;
     
    case ISC_GYRO_CALIBRATION:
    calibratingG = 400;
    GYRO_SAMPLES_TO_CALIBRATE = calibratingG;
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
  uint8_t t = headTX;
  if (++t >= TX_BUFFER_SIZE) t = 0;
  bufTX[t] = a;
  checksum ^= a;
  headTX = t;
}


  ISR_UART {
    uint8_t t = tailTX;
    if (headTX != t) {
      if (++t >= TX_BUFFER_SIZE) t = 0;
      UDR0 = bufTX[t];  // Transmit next byte in the ring
      tailTX = t;
    }
    if (t == headTX) UCSR0B &= ~(1<<UDRIE0); // Check if all data is transmitted . if yes disable transmitter UDRE interrupt
  }


void UartSendData() {
    UCSR0B |= (1<<UDRIE0); // enable transmitter UDRE interrupt if deactivacted
}

static void inline SerialOpen(uint8_t port, uint32_t baud) {
  uint8_t h = ((F_CPU  / 4 / baud -1) / 2) >> 8;
  uint8_t l = ((F_CPU  / 4 / baud -1) / 2);
  switch (port) {

    case 0: UCSR0A  = (1<<U2X0); UBRR0H = h; UBRR0L = l; UCSR0B |= (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0); break;
    case 1: UCSR1A  = (1<<U2X1); UBRR1H = h; UBRR1L = l; UCSR1B |= (1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1); break;
    case 2: UCSR2A  = (1<<U2X2); UBRR2H = h; UBRR2L = l; UCSR2B |= (1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2); break;
    case 3: UCSR3A  = (1<<U2X3); UBRR3H = h; UBRR3L = l; UCSR3B |= (1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3); break;
  }
}

static void inline SerialEnd(uint8_t port) {
  switch (port) {
    case 0: UCSR0B &= ~((1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0)); break;
    case 1: UCSR1B &= ~((1<<RXEN1)|(1<<TXEN1)|(1<<RXCIE1)); break;
    case 2: UCSR2B &= ~((1<<RXEN2)|(1<<TXEN2)|(1<<RXCIE2)); break;
    case 3: UCSR3B &= ~((1<<RXEN3)|(1<<TXEN3)|(1<<RXCIE3)); break;
  }
}

static void inline store_uart_in_buf(uint8_t data, uint8_t portnum) {
  uint8_t h = serialHeadRX[portnum];
  if (++h >= RX_BUFFER_SIZE) h = 0;
  if (h == serialTailRX[portnum]) return; // we did not bite our own tail?
  serialBufferRX[serialHeadRX[portnum]][portnum] = data;
  serialHeadRX[portnum] = h;
}


  ISR(USART1_RX_vect) { store_uart_in_buf(UDR1, 1); }
  ISR(USART0_RX_vect) { store_uart_in_buf(UDR0, 0); }
  ISR(USART2_RX_vect) { store_uart_in_buf(UDR2, 2); }
  ISR(USART3_RX_vect) { store_uart_in_buf(UDR3, 3); }

uint8_t SerialRead(uint8_t port) {
  uint8_t t = serialTailRX[port];
  uint8_t c = serialBufferRX[t][port];
  if (serialHeadRX[port] != t) {
    if (++t >= RX_BUFFER_SIZE) t = 0;
    serialTailRX[port] = t;
  }
  return c;
}

uint8_t SerialAvailable(uint8_t port) {
  return (serialHeadRX[port] != serialTailRX[port]);
}

void SerialWrite(uint8_t port,uint8_t c){
 switch (port) {
    case 0: serialize8(c);UartSendData(); break;                 // Serial0 TX is driven via a buffer and a background intterupt
    case 1: while (!(UCSR1A & (1 << UDRE1))) ; UDR1 = c; break;  // Serial1 Serial2 and Serial3 TX are not driven via interrupts
    case 2: while (!(UCSR2A & (1 << UDRE2))) ; UDR2 = c; break;
    case 3: while (!(UCSR3A & (1 << UDRE3))) ; UDR3 = c; break;
  }
}
