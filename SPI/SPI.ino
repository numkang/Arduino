#define READ 0x00
#define WRITE 0x01
#define ss 53
#include<SPI.h>

signed int temp[4];


void setup(){
  SPI.begin();
  Serial.begin(9600);
  pinMode(ss,OUTPUT);
  digitalWrite(ss,HIGH);
  SPI.setDataMode(SPI_MODE3);  // CPOL = 0, CPHA = 0
  SPI.setClockDivider(SPI_CLOCK_DIV8); //t_clk = 4.00us
  SPI.setBitOrder(MSBFIRST);
}

void loop()
{
      
    digitalWrite(ss, LOW);
    delayMicroseconds(10);
    SPI.transfer(READ);
    SPI.transfer(0x24);
    temp[3] = SPI.transfer(0x00);
    temp[2] = SPI.transfer(0x00);
    temp[1] = SPI.transfer(0x00);
    temp[0] = SPI.transfer(0x00);
    digitalWrite(ss, HIGH);
    int temp1 = temp[3];
    int temp2 = temp1 << 8;
    temp2 = temp2 + temp[2];
    float accel_z = temp2;
    Serial.println(accel_z);
  
}
