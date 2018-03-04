void setup() {
  // put your setup code here, to run once:
  
  pinMode(9, OUTPUT); //pwm wrtie
  pinMode(10, OUTPUT);
  analogWrite(9, 900);
  analogWrite(10, 900);
  //delay(3000);
  
  Serial.begin(115200);
  Wire.begin();
  imu_init();
  
  /*pinMode(5, INPUT); //pwm read
  pinMode(6, INPUT);*/  
}

void loop() {
  // put your main code here, to run repeatedly:
  previousTime = currentTime;
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  
  if(currentTime > taskTime400){ //400 Hz task
    taskTime400 = currentTime + 2500;    
    gyro_getdata();
    filter_gyro();
    
    acc_getdata();
    filter_acc();
    
    com_filter();
    
    //pwm_read();
    pid();
    pwm_write();
    
    SerialRead();
  }
  
  SerialPrint(10);
}
