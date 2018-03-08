#include "HX711.h"

#define DOUT1  3 //Orange
#define CLK1  2 //Yellow
#define DOUT2  5 //Orange
#define CLK2  4 //Yellow

HX711 scale1(DOUT1, CLK1); 
HX711 scale2(DOUT2, CLK2); 

float calibration_factor1 = 23.5; //49
float calibration_factor2 = 24.3; //47.5

int dataIn = 11; //Blue
int clockIn = 12; //Purple
int clock = 1;
int lastClock = 1;
int out = 0;
int i = 0;
int bin_data[24];
float val = 0;

void setup() {
//  pinMode(13,OUTPUT);
  pinMode(dataIn, INPUT);     
  pinMode(clockIn, INPUT);  
  Serial.begin(115200);

  scale1.set_scale();
  scale1.tare();  //Reset the scale1 to 0
  scale1.set_scale(calibration_factor1);
  scale2.set_scale();
  scale2.tare();  //Reset the scale1 to 0
  scale2.set_scale(calibration_factor2);

//  digitalWrite(13,HIGH);
}

void loop(){
  lastClock = clock;
  clock = digitalRead(clockIn);
  
  if(i == 24){
    i = 0;
    val = bin_to_dec(bin_data);
    Serial.print(millis());
//    Serial.print(0);
    Serial.print(" ");
    Serial.print(val);
    Serial.print(" ");
    Serial.print(scale1.get_units());
    Serial.print(" ");
    Serial.print(scale2.get_units());
    Serial.print(" ");
    Serial.println("---------------------");
  }

  if (lastClock == 1 && clock == 0){
    // Tripple sampling to remove glitches
    out = digitalRead(dataIn) + digitalRead(dataIn) + digitalRead(dataIn);
    if (out > 1){
      bin_data[i] = 0;
    }
    else{
      bin_data[i] = 1;
    }      
    i++;
  }
}

float bin_to_dec(int bin[]){
  float value = 0;
  for(int k = 15; k >= 0; k--){
    value = value + bin[k]*pow(2,k);
  }
  val = value/100.0;
  return value/100.0;
}

