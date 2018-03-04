static uint32_t printTime;

void SerialPrint(uint32_t ms){  
  if(currentTime > printTime){
    printTime = currentTime + ms*1000;
    SerialPrintText();
  }
}

void SerialPrintText(){
  Print(0,potenVal); PrintTab(0); Print(0,angle);// PrintTab(0); Print(0,error);
  //Print(0,angle); PrintTab(0); Print(0,error); PrintTab(0); Print(0,output);
  Println(0);
}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                             Serial Print Function                        //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

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
}
