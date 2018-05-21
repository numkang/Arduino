#include <hx711.h>

#define pin_dout A1
#define pin_slk A0
#define avg_num 100

byte data[3];
uint32_t val;
uint16_t val2;
float gram;
float avg = 0;

float offset = 8044;
float w1 = 8434; //output from val2
float w2 = 10.0; //known weight 10g
float ratio;

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  
  pinMode(pin_slk, OUTPUT);
  pinMode(pin_dout, INPUT);

  digitalWrite(pin_slk, HIGH);
  delayMicroseconds(100);
  digitalWrite(pin_slk, LOW);

  for(int i = 0; i < avg_num; i++){
//    Serial.println("aaa");
  while (digitalRead(pin_dout))
    ;

  for (byte j = 3; j--;)
  {
    for (char i = 8; i--;)
    {
      digitalWrite(pin_slk, HIGH);
      bitWrite(data[j], i, digitalRead(pin_dout));
      digitalWrite(pin_slk, LOW);
    }
  }

  digitalWrite(pin_slk, HIGH);
  digitalWrite(pin_slk, LOW);

  data[2] ^= 0x80;

  val = ((uint32_t) data[2] << 16) | ((uint32_t) data[1] << 8)
      | (uint32_t) data[0];

  val2 = val/1000;
  avg = avg + val2;
  }
  offset = avg/avg_num;

  ratio = (w1-offset)/w2;
  Serial.println(ratio);

  Serial.println("Start");
}

void loop() {
  

  while (digitalRead(pin_dout))
    ;

  for (byte j = 3; j--;)
  {
    for (char i = 8; i--;)
    {
      digitalWrite(pin_slk, HIGH);
      bitWrite(data[j], i, digitalRead(pin_dout));
      digitalWrite(pin_slk, LOW);
    }
  }

  digitalWrite(pin_slk, HIGH);
  digitalWrite(pin_slk, LOW);

  data[2] ^= 0x80;

  val = ((uint32_t) data[2] << 16) | ((uint32_t) data[1] << 8)
      | (uint32_t) data[0];

  val2 = val/1000;
  gram = (val2 - offset)/ratio;

//  Serial.print(val);
//  Serial.print(" , ");
  Serial.print(val2);
  Serial.print(" , ");
  Serial.print(gram);
  Serial.println(" grams");
  delay(200);
}
