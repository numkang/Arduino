static uint32_t currentTime = 0;
static uint32_t previousTime = 0;
static uint16_t cycleTime = 0;

static uint32_t interval_1Hz = 0;
static uint32_t interval_2Hz = 0;
static uint32_t interval_4Hz = 0;
static uint32_t interval_5Hz = 0;
static uint32_t interval_10Hz = 0;
static uint32_t interval_20Hz = 0;
static uint32_t interval_50Hz = 0;
// 1/(Hz) * 1000000;

void setup() {
  // put your setup code here, to run once:
  previousTime = micros(); 
}

void loop() {
  // put your main code here, to run repeatedly:
  if(currentTime > interval_1Hz){ //1 Hz task
    interval_1Hz = currentTime + 1000000;
    
  }

  if(currentTime > interval_2Hz){ //2 Hz task
    interval_2Hz = currentTime + 500000;
    
  }

  if(currentTime > interval_4Hz){ //4 Hz task
    interval_4Hz = currentTime + 250000;
    
  }

  if(currentTime > interval_5Hz){ //5 Hz task
    interval_5Hz = currentTime + 200000;
    
  }
  
  if(currentTime > interval_10Hz){ //10 Hz task
    interval_10Hz = currentTime + 100000;
    
  }

  if(currentTime > interval_20Hz){ //20 Hz task
    interval_20Hz = currentTime + 50000;
    
  }
  
  if(currentTime > interval_50Hz){ //50 Hz task
    interval_50Hz = currentTime + 20000;
    
  }

  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime; 
}
