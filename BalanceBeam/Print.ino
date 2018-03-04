static uint32_t printTime;

void SerialPrint(uint32_t ms){  
  if(currentTime > printTime){
    printTime = currentTime + ms*1000;
    //SerialPrintText();
    Serial.println(angle[PITCH]);
  }
}

void SerialPrintText(){
  //Println(0,angle[PITCH]);
  /*Print(0,"Time: ");  PrintTab(0); Println(0,cycleTime); 
  Print(0,"PID1: ");  PrintTab(0); Print(0,angle[PITCH]);    PrintTab(0); Print(0,error);             PrintTab(0); Println(0,errorI); 
  Print(0,"PID2: ");  PrintTab(0); Print(0,errorD);         PrintTab(0); Print(0,previous_error);    PrintTab(0); Println(0,error-previous_error);
  Print(0,"PID3: ");  PrintTab(0); Print(0,Pterm);          PrintTab(0); Print(0,Iterm);             PrintTab(0); Println(0,Dterm);
  Print(0,"OUT: ");   PrintTab(0); Print(0,PID_CMD[0]);     PrintTab(0); Print(0,PWM_Command[M1]);   PrintTab(0); Println(0,PWM_Command[M2]);
  Println(0);*/ 
    /*Print(0,"GYRO: ");   PrintTab(0);Print(0,"X: ");  PrintTab(0);Print(0,gyroRAW[ROLL]);   PrintTab(0);Print(0,"Y: ");  PrintTab(0);Print(0,gyroRAW[PITCH]);  PrintTab(0);Print(0,"Z: ");  PrintTab(0);Println(0,gyroRAW[YAW]);
    Print(0,"Acc: ");    PrintTab(0);Print(0,"X: ");  PrintTab(0);Print(0,accRAW[PITCH]);   PrintTab(0);Print(0,"Y: ");  PrintTab(0);Print(0,accRAW[ROLL]);    PrintTab(0);Print(0,"Z: ");  PrintTab(0);Println(0,accRAW[YAW]);
    Print(0,"Angle: ");  PrintTab(0);Print(0,"ROLL: ");  PrintTab(0);Print(0,(int16_t)angle[PITCH]);    PrintTab(0);Print(0,"PITCH: ");  PrintTab(0);Print(0,(int16_t)angle[ROLL]);
    Println(0);  Println(0); */
  
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                             Serial Print Function                        //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
/*
void Print(uint8_t port, uint8_t a){
  String c = (String)(a);
   for(uint8_t i=0; i<c.length(); i++){
     SerialWrite(port, c[i]);
   }  
}
void Print(uint8_t port, int16_t a){
  String c = (String)(a);
   for(uint8_t i=0; i<c.length(); i++){
     SerialWrite(port, c[i]);
   }  
}

void Print(uint8_t port, uint16_t a){
  String c = (String)(a);
   for(uint8_t i=0; i<c.length(); i++){
     SerialWrite(port, c[i]);
   }  
}

void Print(uint8_t port, int32_t a){
  String c = (String)(a);
   for(uint8_t i=0; i<c.length(); i++){
     SerialWrite(port, c[i]);
   }  
}

void Print(uint8_t port, uint32_t a){
  String c = (String)(a);
   for(uint8_t i=0; i<c.length(); i++){
     SerialWrite(port, c[i]);
   }  
}

void Print(uint8_t port, float a){
  if (a > 4294967040.0) Print(port, "ovf");
  else if (a <-4294967040.0) Print (port, "ovf");
  else{
    int32_t c;
    Print(port, (int32_t)a);
    Print(port, '.');
    for(uint8_t i=0; i<7; i++){
      a*=10;
      c = (int32_t)a;
      Print(port, abs(c%10));
    } 
  }
}

void Print(uint8_t port, double a){
  if (a > 4294967040.0) Print(port, "ovf");
  else if (a <-4294967040.0) Print (port, "ovf");
  else{
    int32_t c;
    Print(port, (int32_t)a);
    Print(port, '.');
    for(uint8_t i=0; i<7; i++){
      a*=10;
      c = (int32_t)a;
      Print(port, abs(c%10));
    } 
  }
}

void Print(uint8_t port, char c){
  SerialWrite(port, c);
}

void Print(uint8_t port, String c){
   for(uint8_t i=0; i<c.length(); i++){
     SerialWrite(port, c[i]);
   }  
}

///////////////////////////////////////////////////////////////////////

void Println(uint8_t port){
   SerialWrite(port, 10); 
}

void Println(uint8_t port, uint8_t c){
   Print(port, c);
   SerialWrite(port, 10); 
}

void Println(uint8_t port, int16_t c){
   Print(port, c);
   SerialWrite(port, 10); 
}

void Println(uint8_t port, uint16_t c){
   Print(port, c);
   SerialWrite(port, 10); 
}

void Println(uint8_t port, int32_t c){
   Print(port, c);
   SerialWrite(port, 10); 
}

void Println(uint8_t port, uint32_t c){
   Print(port, c);
   SerialWrite(port, 10); 
}

void Println(uint8_t port, float c){
   Print(port, c);
   SerialWrite(port, 10); 
}

void Println(uint8_t port, double c){
   Print(port, c);
   SerialWrite(port, 10); 
}

void Println(uint8_t port, char c){
   Print(port, c);
   SerialWrite(port, 10); 
}

void Println(uint8_t port, String c){
   Print(port, c);
   SerialWrite(port, 10); 
}

///////////////////////////////////////////////////////////////////////

void PrintTab(uint8_t port){
  SerialWrite(port, 9);  
}*/
