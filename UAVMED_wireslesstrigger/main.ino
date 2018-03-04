void setup() {
  switch_init();
  PWM_init();
  relay_init();
  delay(12000);
  Serial.begin(115200);
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void loop() {
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

  if(currentTime > taskTime1s){
    taskTime1s = currentTime + 1000000;
    
    if(mission == 0 || mission == 3){
      if(RC_Command[remote_trig] > 1800) relay_off(RELAY_3); //red on
      else relay_on(RELAY_3); //red off
    }
    
    if(RC_Command[board_trig] > 1800){
      if(read_switch(SWITCH_PIN) && pin_switch <= 10 && pin_switch != 0){
        if(pin_switch % 2 == 1) relay_on(RELAY_3); //red blink
        else relay_off(RELAY_3); //red blink
        pin_switch++;
      }
      else if(read_switch(SWITCH_PIN) && pin_switch >= 10){
        relay_off(RELAY_3); //red on
        relay_off(RELAY_4); //green off
        mission = 2; //take off again
      }
      else if(pin_switch != 0){
        relay_on(RELAY_3); //red off
        relay_on(RELAY_4); //green on
        //pin_switch = 1;
        mission = 1; //motor stop
      }
    }
  }

  if(currentTime > taskTime50Hz){ //50 Hz task
    taskTime50Hz = currentTime + 20000;    
    computeRC();
  }

  if(RC_Command[board_trig] > 1800 && mission == 0){
    mission = 1;    
    PWM_write(CH8_Brake, PWM_HIGH);
    delay(10);
    relay_on(RELAY_2);
    delay(1000);
    PWM_write(CH7_MotorInterlock, PWM_LOW);
    delay(10);
    relay_on(RELAY_1);
  }
  else if(mission == 2){
    mission = 3;
    pin_switch = 0;
    //delay(10000);
    for(uint8_t i=0; i<10; i++){
      if(i%2==0) relay_on(RELAY_3); //red blink
      else relay_off(RELAY_3); //red blink
      delay(2500);
    }
    PWM_write(CH7_MotorInterlock, PWM_HIGH);    
    delay(10);
    relay_off(RELAY_1);
    delay(5000);
    PWM_write(CH8_Brake, PWM_LOW);
    delay(10);
    relay_off(RELAY_2);
  }
  else{
    PWM_write(CH7_MotorInterlock, RC_Command[remote_trig]);
  }
  Serial.print(mission);
  Serial.println(read_switch(SWITCH_PIN));
}
