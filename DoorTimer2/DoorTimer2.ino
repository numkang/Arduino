//#include <WProgram.h>
#include <Wire.h>
#include "RTClib.h"
#include <avr/wdt.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

#define PIN_DOOR_CTRL 13
#define STATE_KEY_OPR     HIGH
#define STATE_KEY_NOP     LOW

/*Time struct config Ex. 8AM config 800,8.30AM config 830,3.30PM config 1530 */
/*NOP = No operate it's mean KEY INACTIVE*/
/*OPR = Operate it's mean KEY ACTIVE*/
typedef struct
{
  unsigned int time_nop;
  unsigned int time_opr;
}Rule;

void KeyCardInActive(void );
void KeyCardActive(void );

int rtc[7];
RTC_DS1307 RTC;

Rule rule[7];

void setup()
{
   // set up the LCD's number of rows and columns: 
  lcd.begin(20, 4);
 
  wdt_enable(WDTO_500MS);          //Set WatchDog at 500ms
  pinMode(PIN_DOOR_CTRL, OUTPUT);  //Set Pin output for Control Door
  
  //Sunday is lock all day.
  rule[0].time_nop = 2500;
  rule[0].time_opr = 0000;
  
  //Monday is open 0830H to 1630H.
  rule[1].time_nop = 830;
  rule[1].time_opr = 1630;
    
  //Tuesday is open 0830H to 1630H.
  rule[2].time_nop = 830;
  rule[2].time_opr = 1630;
  
  //Wednesday is open 0830H to 1630H.
  rule[3].time_nop = 830;
  rule[3].time_opr = 1630;
  
  //Thursday is open 0830H to 1630H.
  rule[4].time_nop = 830;
  rule[4].time_opr = 1630;
  
  //Friday is open 0830H to 1630H.
  rule[5].time_nop = 830;
  rule[5].time_opr = 1630;
  
  //Saturday is lock all day.
  rule[6].time_nop = 2500;
  rule[6].time_opr = 0000;
  
  DDRC  |= _BV(2) | _BV(3);  // POWER:Vcc Gnd
  PORTC |= _BV(3);  // VCC PINC3
   
  Serial.begin(9600);
  Wire.begin();
  RTC.begin();
  //RTC.adjust(DateTime(__DATE__, __TIME__));
  
  /*Config Squard Wave*/
 // 
// Wire.beginTransmission(DS1307_CTRL_ID);
//  Wire.send(0x07);  // CONTROL register pointer
//  Wire.send(0x10);  // 1Hz 
//  Wire.endTransmission();
  
  /*Setup Time only, Don't use when Normal work*/
//  RTC.stop();
//  RTC.set(DS1307_SEC,0);
//  RTC.set(DS1307_MIN,23);
//  RTC.set(DS1307_HR,14);
//  RTC.set(DS1307_DOW,3);  //1=Sunday, 2=Monday,...,7=Saterday
//  RTC.set(DS1307_DATE,15);
//  RTC.set(DS1307_MTH,9);
//  RTC.set(DS1307_YR,15);
//  RTC.start();
}

void loop()
{
  DateTime now = RTC.now();
   /*READ Time From RTC*/
  //RTC.get(rtc,true);
  
  /*Check Time; TIME_OPEN < TIME < TIME_CLOSE*/
  //if(((rtc[2]*100 + rtc[1] ) >= rule[rtc[3]-1].time_nop) && ((rtc[2]*100 + rtc[1] ) < rule[rtc[3]-1].time_opr))  //When RTC is while time of door open
  //Rule count 0-6; 0 is Sunday, 6 is Saterday; but RTC count 1-7; 1 is sunday ,7 is Saterday; so minus(1) from RTC[3] = Day of rule
  /*{
    KeyCardInActive();      
    Serial.print("KEY CARD INACTIVE"); 
     lcd.setCursor(0, 1);  
     lcd.print("KEY INACTIVE"); 
  }
  else
  {
    KeyCardActive();    //Door Lock
    Serial.print("Key Card Active");
    lcd.setCursor(0, 1);
     lcd.print("KEY ACTIVE"); 
     }*/

  if(now.dayOfWeek() == 0 || now.dayOfWeek() == 6)
  {
    KeyCardActive();    //Door Lock
    Serial.print("Key Card Active");
    lcd.setCursor(0, 1);
    lcd.print("KEY ACTIVE");   
  }
  else{
    if(now.hour() >= 6 && now.hour() <= 17){
      KeyCardInActive();      
      Serial.print("KEY CARD INACTIVE"); 
      lcd.setCursor(0, 1);  
      lcd.print("KEY INACTIVE");
    }
    else{
      KeyCardActive();    //Door Lock
      Serial.print("Key Card Active");
      lcd.setCursor(0, 1);
      lcd.print("KEY ACTIVE"); 
    }
  }
    
 /*Serial.print(rtc[2], DEC);
 Serial.print(":");   
 Serial.print(rtc[1], DEC);
 Serial.print(":");
 Serial.print(rtc[0], DEC);
 Serial.print(" ");
 
 Serial.print(rtc[3], DEC);
 Serial.print(" ");
 Serial.print(rtc[4], DEC);
 Serial.print(" ");
 Serial.print(rtc[5], DEC);
 Serial.print(" ");
 Serial.print(rtc[6], DEC);
 Serial.print(" ");
 
 Serial.print(rule[rtc[3]].time_nop, DEC);
 Serial.print(" ");
 Serial.println(rule[rtc[3]].time_opr, DEC);*/

 
 wdt_reset(); //Watchdog Reset 
  
{
  /*// set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 0);  
// lcd.print(rtc[7], DEC);
 
  lcd.setCursor(0, 1);  
 lcd.print(rtc[2], DEC);
 lcd.print(":"); 
 lcd.print(rtc[1], DEC);
 lcd.print(":");   
 lcd.print(rtc[0], DEC);
 lcd.print(" "); 
 
  lcd.setCursor(0, 2); 
 lcd.print(rtc[3], DEC);
 lcd.print("/");
 lcd.print(rtc[4], DEC);
 lcd.print("/");
 lcd.print(rtc[5], DEC);
 lcd.print("/");
 lcd.print(rtc[6], DEC);
  
 //lcd.setCursor(1, 1);  */

 // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  //lcd.setCursor(0, 0);  
// lcd.print(rtc[7], DEC);
 
  lcd.setCursor(0, 0);
  lcd.print(now.dayOfWeek(), DEC);
  lcd.print("/");
  lcd.print(now.day(), DEC);
 lcd.print("/");
 lcd.print(now.month(), DEC);
 lcd.print("/");
 lcd.print(now.year()-2000, DEC);
 lcd.print(" ");  
 lcd.print(now.hour(), DEC);
 lcd.print(":");   
 if(now.minute() < 10){
 lcd.print("0"); 
 }
 lcd.print(now.minute(), DEC);

 //lcd.print(":");   
 //lcd.print(now.second(), DEC);
 
  //lcd.setCursor(0, 1); 
 //lcd.print(rtc[3], DEC);
 //lcd.print("/");
 
  
 //lcd.setCursor(1, 1);
 
delay(100);
}
 
}
void KeyCardInActive(void )
{
  digitalWrite(PIN_DOOR_CTRL, STATE_KEY_NOP);
}

void KeyCardActive(void )
{
  digitalWrite(PIN_DOOR_CTRL, STATE_KEY_OPR);
}

