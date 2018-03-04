#include <Wire.h>
static uint8_t nunchuck_buf[6]; 
#define pwrpin PORTC3
#define gndpin PORTC2

int loop_cnt = 0;

void setup(){
  Serial.begin(115200);
  Wire.begin();
  nunchuck_setpowerpins();
  Serial.println(PORTC,BIN);
  nunchuck_init(); // send the initilization handshake
  Serial.println("begin2");
}

void loop(){
  if( loop_cnt > 100 ) { // every 100 msecs get new data
    loop_cnt = 0;
    
    nunchunk_update();
            
    for(int i=0; i<5; i++){
      Serial.print(nunchuck_buf[i]);
      Serial.print(" , ");
    }
    Serial.print(nunchuck_zbutton());
    Serial.print(" , ");
    Serial.print(nunchuck_cbutton());
    Serial.println();
  }
  loop_cnt++;
  delay(1);
}

