#include "config.h"
#include "definition.h"



//static uint16_t last_time,now;
static uint16_t calibratingA = 400;  
static uint16_t calibratingG = 400;
static uint16_t acc_1G;             
static int16_t  acc_25deg;
//static int16_t  i2c_errors_count = 0;
static int16_t  gyroADC[3],accADC[3],accSmooth[3],magADC[3];
static int16_t gyroData[3] = {0,0,0};
static int16_t gyroZero[3] = {0,0,0};
static int16_t angle[2]    = {0,0};

static int16_t  heading;
static float   magCal[3] = {1.0,1.0,1.0};  // gain for each axis, populated at sensor init
static uint8_t magInit = 0;
static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t GYRO_SAMPLES_TO_CALIBRATE = 600;
static uint16_t ACC_SAMPLES_TO_CALIBRATE = 400;
static uint16_t pPWM[3] = {1000,1000,1000};
static uint16_t motorCMD[3] = {1000,1000,1000};
static int16_t angle_cmd = 0;

static int16_t angle_diff,angle_i;
static int16_t PID_cmd;
static uint16_t PIDgains[3];


struct flags_struct {
  uint8_t gyroCalibrated :1 ;
  uint8_t accCalibrated :1 ;
  uint8_t SMALL_ANGLES_25 :1 ;
  uint8_t sensorsCalibrated :1 ;
  uint8_t CALIBRATE_MAG :1 ;
  uint8_t CTRL_MODE :1 ; //0 = NORMAL PID, 1 = MANUAL PWM MODE
  uint8_t SetpointHeading :1 ;
  uint8_t PotSampled :1;
  uint8_t PotReadCount;
} f;

static struct {
  int16_t accZero[3];
  int16_t magZero[3];
} conf;




void setup() {

pinMode(13,OUTPUT);
for(uint8_t tt = 0;tt < 10;tt++){
TOGGLE_LEDPIN delay(50);
}
for(uint8_t tt = 0;tt < 30;tt++){
TOGGLE_LEDPIN delay(20);
}

i2c_init();
initSensors();
delay(100);
SerialOpen(0,SERIAL_COM_SPEED);
digitalWrite(13,LOW);
initPWM();

f.CTRL_MODE = NORMAL;
#if defined(CALIBRATE_MOTOR)
OCR3B = PWMscale(2000);
OCR3C = PWMscale(2000);
delay(4000);
while(1){
OCR3B = PWMscale(1000);
OCR3C = PWMscale(1000);
}
#endif
// Default Gains
PIDgains[P] = 10;
PIDgains[I] = 1;
PIDgains[D] = 80;
}

void loop() {

if(f.accCalibrated == 1 && f.gyroCalibrated == 1 && f.sensorsCalibrated == 0)
{ f.sensorsCalibrated = 1;}
else if(f.sensorsCalibrated == 1) {Mag_getADC();currentTime = micros();computeIMU();}
else { Gyro_getADC(); ACC_getADC(); }


PIDc();
serialCom();
updateDutyCycle();

}


