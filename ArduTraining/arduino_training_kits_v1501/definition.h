#define ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  = -X; accADC[PITCH]  = -Y; accADC[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] =  Y; gyroADC[PITCH] = -X; gyroADC[YAW] = -Z;}
#define MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = -Z;}
#define ADXL345_ADDRESS 0x53
#define I2C_SPEED 400000L
#define L3G4200D_ADDRESS 0x69
#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03
#define MAG_VALUE magADC[axis]  
#define LEDPIN_TOGGLE PINB |= 1<<5;
#define TOGGLE_LEDPIN digitalWrite(13,!digitalRead(13)); 
#define ROLL       0
#define PITCH      1
#define YAW        2
#define ACC_LPF_FACTOR 100
#define GYR_CMPF_FACTOR 400.0f
#define GYR_CMPFM_FACTOR 200.0f
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#define GYRO_SCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f))
#define ssin(val) (val)
#define scos(val) 1.0f

#define PWMscale(x) x<<3

#define  VERSION  191
#define MULTITYPE 11
#define cycleTime 3000
#define ACC 1
#define BARO 0
#define MAG 0
#define GPS 0
#define SONAR 0
#define ISR_UART ISR(USART0_UDRE_vect)

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
#define PROMINI
#endif

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define MEGA
#endif

#define M1 0
#define M2 1

#define P 0
#define I 1
#define D 2

#define EL_POT A0
#define PL_POT A1
#define FORWARD 1
#define BACKWARD 0

#define NORMAL 0
#define MANUAL 1
