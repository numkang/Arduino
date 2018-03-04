static uint32_t printTime;

void SerialPrint(uint32_t ms){  
  if(currentTime > printTime){
    printTime = currentTime + ms*1000;
    SerialPrintText();
  }
}

void SerialPrintText(){
  Print(0,"RC1: ");  PrintTab(0); Println(0,RC_Command[ROLL]);
  Print(0,"RC2: ");  PrintTab(0); Println(0,RC_Command[PITCH]);
  Print(0,"RC3: ");  PrintTab(0); Println(0,RC_Command[THR]);
  Print(0,"RC4: ");  PrintTab(0); Println(0,RC_Command[YAW]);
  Print(0,"RC5: ");  PrintTab(0); Println(0,RC_Command[AUX1]);
  Print(0,"RC6: ");  PrintTab(0); Println(0,RC_Command[AUX2]);
  Print(0,"RC7: ");  PrintTab(0); Println(0,RC_Command[AUX3]);
  Print(0,"RC8: ");  PrintTab(0); Println(0,RC_Command[AUX4]);
  Println(0);
}
