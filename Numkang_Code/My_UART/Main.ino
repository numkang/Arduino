//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                 Main function                            //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
    
void setup() {
  previousTime = micros();
  SerialOpen(0,115200);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  digitalWrite(12, 1);
}

void loop() {  
  if(currentTime > taskTime50){ //50 Hz task
    taskTime50 = currentTime + 20000;
    
    //Println(0,"Hello World");    
  }
  digitalWrite(12, 1);
  char c;
  while(SerialAvailable(0)){
    c = SerialRead(0);
    Println(0, c);
    
    switch(c) {
      case '0' :
      digitalWrite(13, 0);
      Print(0,"OFF");
      break;
      
      case '1' :
      digitalWrite(13, 1);
      Print(0,"ON");
      break;
      
      case 'i' :
      digitalWrite(13, !digitalRead(13));
      Print(0,"FUCK");
      break;
      
      default :
      break;
  }
  }
  
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;
}
