#include <hx711.h>
//Thanakorn
#define Pin_dout 3
#define Pin_slk 2
#define avg_num 100

byte data[3];
uint32_t val;
uint16_t val2;
float gram;
float avg = 0;

float force_offset; //8044;
//float w1 = 8434; //output from val2
//float w2 = 10.0; //known weight in grams
//float force_ratio;

#define Pin_pos A0

static uint32_t currentTime = 0;
static uint32_t previousTime = 0;
static uint16_t cycleTime = 0;
static uint32_t taskTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  force_setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;
  
  sensor_loop();
  int pos_sensorValue = analogRead(Pin_pos);

  if(currentTime > taskTime){ //100 Hz task
    taskTime = currentTime + 10000;
    Serial.print(pos_sensorValue);
    Serial.print(",");
    Serial.println(val2);
  }
}
