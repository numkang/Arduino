#include "HX711.h"

#define DOUT1  3
#define CLK1  2
#define DOUT2  5
#define CLK2  4
//#define DOUT3  7
//#define CLK3  6

HX711 scale1(DOUT1, CLK1);
HX711 scale2(DOUT2, CLK2);
//HX711 scale3(DOUT3, CLK3);

float calibration_factor1 = 45; //450 
float calibration_factor2 = 43.5; //1440
//float calibration_factor3 = 1440; 

void setup() 
{
  Serial.begin(115200);
  scale1.set_scale();
  scale1.tare();  //Reset the scale1 to 0
  scale1.set_scale(calibration_factor1);
  scale2.set_scale();
  scale2.tare();  //Reset the scale1 to 0
  scale2.set_scale(calibration_factor2);
  //scale3.set_scale();
  //scale3.tare();  //Reset the scale1 to 0
  //scale3.set_scale(calibration_factor3);
}

void loop()
{
  Serial.print(millis());
  Serial.print(" ");
  Serial.print("13");
  Serial.print(" ");
  Serial.print(scale1.get_units());
  Serial.print(" ");
  Serial.println(scale2.get_units());
  delay(100);
}
