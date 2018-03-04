static uint32_t printTime;

void SerialPrint(uint32_t ms){  
  if(currentTime > printTime){
    printTime = currentTime + ms*1000;
    SerialPrintText();
  }
}

void SerialPrintText(){
    Print(0,"GYRO: ");   PrintTab(0);Print(0,"X: ");  PrintTab(0);Print(0,gyroRAW[ROLL]);   PrintTab(0);Print(0,"Y: ");  PrintTab(0);Print(0,gyroRAW[PITCH]);  PrintTab(0);Print(0,"Z: ");  PrintTab(0);Println(0,gyroRAW[YAW]);
    Print(0,"Acc: ");    PrintTab(0);Print(0,"X: ");  PrintTab(0);Print(0,accRAW[PITCH]);   PrintTab(0);Print(0,"Y: ");  PrintTab(0);Print(0,accRAW[ROLL]);    PrintTab(0);Print(0,"Z: ");  PrintTab(0);Println(0,accRAW[YAW]);
    Print(0,"Angle: ");  PrintTab(0);Print(0,(int16_t)angle[PITCH]);
    Println(0);  Println(0);
}
