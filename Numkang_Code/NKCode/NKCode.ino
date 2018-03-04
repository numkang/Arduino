//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//                                                                                      //
//                                                                                      //
//                     Numkang's code for Microcontroller                               //
//                      SOFTWARE WRITTEN BY NUMKANG_AE20                                //
//          DEPARTMENT OF AEROSPACE ENGINEERING, KASETSART UNIVERSITY                   //
//                                                                                      //
//      This program is free software: you can redistribute it and/or modify            //
//      it under the terms of the GNU General Public License as published by            //
//      the Free Software Foundation, either version 3 of the License, or               //
//      any later version. see <http://www.gnu.org/licenses/>                           //
//                                                                                      //  
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

  /****************************    The type of aircraft    ******************************/    
    #define QUADX 0
    //#define HEXX 1
    //#define MAVion 2
    //#define IFO 4
    //#define useSERVO 4
    
  /***************************    Combined IMU Boards    ********************************/
    //#define AltIMU
    //#define CRIUS_AIO_PRO
    //#define CRIUS_SE_v2 //Arduino Pro Mini
    #define GY_80
    //#define GY_81
    //#define GY_86
    //#define GY_87
    //#define MPU6050
    //#define MPU9150
    //#define NANOWII //Arduino Leonardo
    //#define CHR_UM6
      
    //#define HCSR04 
    
  /***************************    Select Sensor to use    ********************************/   
    //#define useMAG 
    //#define useBARO
    //#define useSONAR //only for Arduino 168/328 echo D8, trig D12      
    //#define useIFRA 
    //#define useGPS  
    //#define useIMG 
    
    #define THROTTLE_ANGLE_CORRECTION 40

  /****************************        Command          *******************************/
    /* this is the value for the ESCs when they are not armed
       in some cases, this value must be lowered down to 900 for some specific ESCs, otherwise they failed to initiate */
    #define MINCOMMAND 950
    #define MAXCOMMAND 1900

/***********************************        End of User's Config          *****************************************/
    
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                 Time function                            //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

static uint32_t currentTime = 0;
static uint32_t previousTime = 0;
static uint16_t cycleTime = 0;
static uint32_t taskTime50 = 0, taskTime4 = 0, taskTime2 = 0, taskTime16 = 0, taskTime500 = 0;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                              AVR Microchips Setup                        //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define UNO
  #define PROMINI
  #define NUMBER_MOTORS 4 //for QUAD
#endif

#if defined(__AVR_ATmega32U4__)
  #define NANO
  #define NUMBER_MOTORS 2 //for MAVion
#endif

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define MEGA
  #define NUMBER_MOTORS 6 //for QUAD or HEX
#endif

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           General function constants                     //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define VER1 2
#define VER2 1
#define VER3 102

#define I2C_SPEED 400000L

#define THR 0

enum rc {
  ROLL,
  PITCH,
  YAW,
  AUX1,
  AUX2,
  AUX3,
  AUX4
};

enum MotorServo {
  M1,
  M2,
  M3,
  M4,
  M5,
  M6,
  M7,
  M8,
  S1,
  S2,
  S3
};

struct function_struct {
  uint8_t gyroCalibrated :1 ;
  uint8_t accCalibrated :1 ;
  uint8_t magCalibrated :1 ;
  
  uint8_t motorCalibrated :1 ;
  
  uint8_t SMALL_ANGLES_25 :1 ;
  
  uint8_t DISARM :1 ;
  uint8_t ARM :1 ;
  
  uint8_t Aircraft_Config ;
  uint8_t Mode_Config ;
  uint8_t Mode_Config_temp ;
  uint8_t ANGLE_MODE :1 ;
  uint8_t HORIZON_MODE :1 ;
  uint8_t RATE_MODE :1 ;
  uint8_t BARO_MODE :1 ;
  uint8_t MOUSE_MODE :1 ;
  uint8_t KINECT_MODE :1 ;  
} f;

#if defined (AltIMU)
  #define sensor 0
#elif defined (CRIUS_AIO_PRO)
  #define sensor 1
#elif defined (CRIUS_SE_v2)
  #define sensor 2
#elif defined (GY_80)
  #define sensor 3
#elif defined (GY_81)
  #define sensor 4
#elif defined (GY_86)
  #define sensor 5
#elif defined (GY_87)
  #define sensor 6
#elif defined (MPU6050)
  #define sensor 7  
#elif defined (MPU9150)
  #define sensor 8
#elif defined (NANOWII)
  #define sensor 9
#elif defined (CHR_UM6)
  #define sensor 10
#endif

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                   LED function                           //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
#define ON_LEDPIN_13 digitalWrite(13,HIGH); //red
#define OFF_LEDPIN_13 digitalWrite(13,LOW);
#define ON_LEDPIN_30 digitalWrite(30,HIGH); //yellow
#define OFF_LEDPIN_30 digitalWrite(30,LOW);
#define ON_LEDPIN_31 digitalWrite(31,HIGH); //orange
#define OFF_LEDPIN_31 digitalWrite(31,LOW);
#define TOGGLE_LEDPIN_13 digitalWrite(13,!digitalRead(13)); 
#define TOGGLE_LEDPIN_30 digitalWrite(30,!digitalRead(30)); 
#define TOGGLE_LEDPIN_31 digitalWrite(31,!digitalRead(31)); 
#define BLINK_LEDPIN_13(T1,T2) for(uint8_t t = 0;t < T1;t++){TOGGLE_LEDPIN_13 delay(T2);}
#define BLINK_LEDPIN_30(T1,T2) for(uint8_t t = 0;t < T1;t++){TOGGLE_LEDPIN_30 delay(T2);}
#define BLINK_LEDPIN_31(T1,T2) for(uint8_t t = 0;t < T1;t++){TOGGLE_LEDPIN_31 delay(T2);}
#define ALL_LED(T1,T2) BLINK_LEDPIN_13(T1,T2)BLINK_LEDPIN_30(T1,T2)BLINK_LEDPIN_31(T1,T2)

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                        Interrupt Constant & Variables                    //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#if defined (UNO) || defined (PROMINI)
  #define RC_CHANS 5
  #define PCINT_PIN_COUNT 5
  #define PCINT_RX_BITS (1<<2),(1<<4),(1<<5),(1<<6),(1<<7)
  #define RX_PCINT_PIN_PORT PIND
#endif

#if defined (NANO)
  #define RC_CHANS 5
  #define PCINT_PIN_COUNT 5
  #define PCINT_RX_BITS (1<<4),(1<<2),(1<<3),(1<<1)
  #define RX_PCINT_PIN_PORT PINB
#endif

#if defined (MEGA)
  #define RC_CHANS 8
  #define PCINT_PIN_COUNT 8
  #define PCINT_RX_BITS (1<<0),(1<<1),(1<<2),(1<<3),(1<<4),(1<<5),(1<<6),(1<<7)
  #define RX_PCINT_PIN_PORT PINK
#endif

static int16_t rcData[RC_CHANS]; //interval [1000;2000]
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS};
volatile uint16_t rcValue[RC_CHANS];
volatile int16_t RC_Command[RC_CHANS];

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                       PWM & PID Constant & Variables                     //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define PID_MIX(X,Y,Z) RC_Command[THR] + X*PID_CMD[ROLL] + Y*PID_CMD[PITCH] + Z*PID_CMD[YAW];
static uint16_t PWM_Command[9];
int32_t prop = 0;
int16_t rc;

static int16_t PTerm;
static int16_t ITerm;
static int16_t DTerm;
static int16_t PTermACC;
static int16_t ITermACC;

static int16_t errorAngle;
static int16_t errorAngleI[3];
static int16_t error;
static int16_t errorGyroI[3];

static int16_t PID_CMD[3];
static int16_t delta,deltaSum,delta1[3],delta2[3];
static int16_t lastGyro[3];

static uint8_t AKp[2]; //Angle Level Mode , PID maybe the same in every axis
static uint8_t AKi[2];
static uint8_t AKd[2];
static uint8_t GKp[3]; //Rate Gyro Mode or Horizon Mode
static uint8_t GKi[3];
static uint8_t GKd[3];

static uint8_t AltKp; //Rate Gyro Mode or Horizon Mode
static uint8_t AltKi;
static uint8_t AltKd;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                      Control Board Constant & Variables                  //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
#if defined(AltIMU)
  #define L3GD20
  #define LSM303DLHC //ACC + MAG
  #define LPS331AP
  #define ACC_ORIENTATION(X, Y, Z)  {accRAW[ROLL]  =  X; accRAW[PITCH]  =  Y; accRAW[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroRAW[ROLL] =  Y; gyroRAW[PITCH] = -X; gyroRAW[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magRAW[ROLL]  =  X; magRAW[PITCH]  =  Y; magRAW[YAW]  =  Z;}
#endif

#if defined(CRIUS_AIO_PRO) 
  #define MPU6050 
  #define HMC5883 
  #define MS561101BA
  #define ACC_ORIENTATION(X, Y, Z)  {accRAW[ROLL]  = -X; accRAW[PITCH]  = -Y; accRAW[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroRAW[ROLL] =  Y; gyroRAW[PITCH] = -X; gyroRAW[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magRAW[ROLL]  =  X; magRAW[PITCH]  =  Y; magRAW[YAW]  = -Z;}
  #define MPU6050_I2C_AUX_MASTER
#endif

#if defined(CRIUS_SE_v2)
  #define MPU6050
  #define HMC5883
  #define BMP085
  #define ACC_ORIENTATION(X, Y, Z)  {accRAW[ROLL]  = -X; accRAW[PITCH]  = -Y; accRAW[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroRAW[ROLL] =  Y; gyroRAW[PITCH] = -X; gyroRAW[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magRAW[ROLL]  =  X; magRAW[PITCH]  =  Y; magRAW[YAW]  = -Z;}
  #define MPU6050_I2C_AUX_MASTER
#endif

#if defined(GY_80)
  #define L3G4200D
  #define ADXL345
  #define HMC5883
  #define BMP085
  #define ACC_ORIENTATION(X, Y, Z)  {accRAW[ROLL]  = -X; accRAW[PITCH]  = -Y; accRAW[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroRAW[ROLL] =  Y; gyroRAW[PITCH] = -X; gyroRAW[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magRAW[ROLL]  =  X; magRAW[PITCH]  =  Y; magRAW[YAW]  = -Z;}
#endif

#if defined(GY_81)
  #define ITG3205
  #define BMA180
  #define HMC5883
  #define BMP085
  #define ACC_ORIENTATION(X, Y, Z)  {accRAW[ROLL]  = -X; accRAW[PITCH]  = -Y; accRAW[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroRAW[ROLL] =  Y; gyroRAW[PITCH] = -X; gyroRAW[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magRAW[ROLL]  =  X; magRAW[PITCH]  =  Y; magRAW[YAW]  = -Z;}
#endif

#if defined(GY_86)
  #define MPU6050 //GYRO + ACC
  #define HMC5883
  #define MS561101BA
  #define ACC_ORIENTATION(X, Y, Z)  {accRAW[ROLL]  = -X; accRAW[PITCH]  = -Y; accRAW[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroRAW[ROLL] =  Y; gyroRAW[PITCH] = -X; gyroRAW[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magRAW[ROLL]  =  X; magRAW[PITCH]  =  Y; magRAW[YAW]  = -Z;}
  #define MPU6050_I2C_AUX_MASTER
#endif

#if defined(GY_87)
  #define MPU6050
  #define HMC5883
  #define BMP085 //BMP085 and BMP180 are software compatible
  #define ACC_ORIENTATION(X, Y, Z)  {accRAW[ROLL]  = -X; accRAW[PITCH]  = -Y; accRAW[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroRAW[ROLL] =  Y; gyroRAW[PITCH] = -X; gyroRAW[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magRAW[ROLL]  =  X; magRAW[PITCH]  =  Y; magRAW[YAW]  = -Z;}
#endif

#if defined(MPU6050)
  #define ACC_ORIENTATION(X, Y, Z)  {accRAW[ROLL]  = -X; accRAW[PITCH]  = -Y; accRAW[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroRAW[ROLL] =  Y; gyroRAW[PITCH] = -X; gyroRAW[YAW] = -Z;}
#endif

#if defined(MPU9150)
  #define MPU6050
  #define AK8975
  #define ACC_ORIENTATION(X, Y, Z)  {accRAW[ROLL]  = -X; accRAW[PITCH]  = -Y; accRAW[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroRAW[ROLL] =  Y; gyroRAW[PITCH] = -X; gyroRAW[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magRAW[ROLL]  =  X; magRAW[PITCH]  =  Y; magRAW[YAW]  = -Z;}
  //#define MPU6050_I2C_AUX_MASTER
#endif

#if defined(NANOWII)
  #define MPU6050
  #define ACC_ORIENTATION(X, Y, Z)  {accRAW[ROLL]  = -Y; accRAW[PITCH]  =  X; accRAW[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroRAW[ROLL] = -X; gyroRAW[PITCH] = -Y; gyroRAW[YAW] = -Z;}
#endif
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                      Sensor Chip Constant & Variables                    //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#if defined(ADXL345) || defined(BMA180) || defined(MMA7361) || defined(MPU6050) || defined(LSM303DLHC) || defined(CHR_UM6)
  #define ACCELEROMETER
#endif

#if defined(L3G4200D) || defined(ITG3205) || defined(MPU6050) || defined(L3GD20) || defined(CHR_UM6)
  #define GYROSCOPE
#endif

#if defined(HMC5883) || defined(AK8975)  || defined(LSM303DLHC) || defined(CHR_UM6)
  #define MAGNETOMETER
#endif

#if defined(BMP085) || defined(MS561101BA) || defined(LPS331AP)
  #define BAROMETER
#endif

#if defined(HCSR04)
  #define ULTRASONIC
#endif

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                    Sensor Address Constant & Variables                   //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
//IMU
#if defined(MPU6050)
  #define MPU6050_ADDRESS 0x68  
  #define MPU6050_DLPF_CFG 2 //0-256 1-188 2-98 3-42 4-20 5-10 6-5
#endif

#if defined(LSM303DLHC)
  #define LSM303DLHC_ACC_ADDRESS 0x19 
  #define LSM303DLHC_MAG_ADDRESS 0x1E
#endif

//ACC
#if defined(ADXL345)
  #define ADXL345_ADDRESS 0x53
#endif

#if defined(MMA7361)
  #define MMA7361_ADDRESS 0x00
#endif

#if defined(BMA180)
  #define BMA180_ADDRESS 0x40
#endif

//GYRO
#if defined(L3G4200D)
  #define L3G4200D_ADDRESS 0x69
#endif

#if defined(L3GD20)
  #define L3GD20_ADDRESS 0x6B
#endif

#if defined(ITG3205)
  #define ITG3205_ADDRESS 0x68
  #define ITG3205_SMPLRT_DIV 0 //8000Hz sample
  #define ITG3205_DLPF_CFG   0 //0-256 1-188 2-98 3-42 4-20 5-10 6-5
#endif

//MAG
#if defined(HMC5883)
  #define HMC5883_ADDRESS 0x1E
  #define MAG_ADDRESS 0x1E
  #define MAG_DATA_REGISTER 0x03
#endif

#if defined(AK8975)  
  #if defined(MPU6050_I2C_AUX_MASTER)
    #define AK8975_ADDRESS 0x0E
    #define MAG_ADDRESS 0x0E
  #else
    #define AK8975_ADDRESS 0x0C
    #define MAG_ADDRESS 0x0C
  #endif
  #define MAG_DATA_REGISTER 0x03  
#endif

//BARO
#if defined(BMP085)
  #define BMP085_ADDRESS 0x77
#endif

#if defined(MS561101BA) 
  #define MS561101BA_ADDRESS 0x77
#endif

#if defined(LPS331AP)
  #define LPS331AP_ADDRESS 0x5D
#endif
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           IMU Constant & Variables                       //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
#if !defined (CHR_UM6)
//I2C
uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;
static int16_t i2c_errors_count = 0;

#if defined(ACCELEROMETER)
  #define ACC_TO_SAMPLE 400
  static uint16_t calibratingA = 400;
  static int16_t accRAW[3], accSmooth[3];
  static int16_t ACC_Zero[3];
#endif

#if defined(GYROSCOPE)
  #define GYRO_TO_SAMPLE 400
  static int16_t gyroRAW[3];
  static uint16_t calibratingG = 400;
  static int16_t GYRO_Zero[3];
  static int16_t gyroData[3] = {0,0,0};
#endif

  static int16_t magRAW[3];
#if defined(MAGNETOMETER) && defined(useMAG)
  #define MAG_DECLINIATION 0.0f
  static int16_t magSmooth[3];
  static float magScale[3] = {1.0,1.0,1.0};
  static uint8_t magInit = 0;
  static int16_t MAG_Zero[3];
#endif
#endif
  
#if defined(BAROMETER) && defined(useBARO)
  #define ALT_HOLD_THROTTLE_NEUTRAL_ZONE 50
  static uint16_t calibratingB = 200;  // baro calibration = get new ground pressure value
  static int32_t EstAlt; // in cm  
  static int32_t baroPressure;
  static int32_t baroTemperature;
  static int32_t baroPressureSum;  
  static int32_t BaroAlt,AltHold; // in cm
  static int16_t BaroPID = 0;
  static int16_t errorAltitudeI = 0;
  static int16_t vario = 0;  
  static int16_t initialThrottleHold;
#endif

#if defined(ULTRASONIC) && defined(useSONAR)
  #define echoPin 8 // Echo Pin
  #define trigPin 12 // Trigger Pin
  static uint32_t duration, distanceCM, distanceINCH;
  uint16_t last_time_sonar;
#endif

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                  Complementary Filter Constant & Variables               //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
static int16_t angle[3] = {0,0,0};
static int16_t heading;

#if !defined (CHR_UM6)
#define ACC_LPF_FACTOR 4 //100

#if defined(MMA7361)
  #define ACC_1G 64
#elif defined(ADXL345)
  #define ACC_1G 265
#elif defined(BMA180)
  #define ACC_1G 255
#elif defined(MPU6050)
  #define ACC_1G 512
#elif defined(LSM330)
  #define ACC_1G 256
#elif defined(LSM303DLHC)
  #define ACC_1G 265 //256
#else
  #define ACC_1G 256
#endif

#define ACCZ_25deg   (int16_t)(ACC_1G * 0.90631) // 0.90631 = cos(25deg) (cos(theta) of accZ comparison)
#define ACC_VelScale (9.80665f / 10000.0f / ACC_1G)

#define GYR_CMPFA_FACTOR 600.0f //400
#define GYR_CMPFM_FACTOR 250.0f //200,300
#define INV_GYR_CMPFA_FACTOR (1.0f / (GYR_CMPFA_FACTOR + 1.0f))
#define INV_GYR_CMPFM_FACTOR (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

#if defined(ITG3200)
  #define GYRO_SCALE (4 / 14.375 * PI / 180.0 / 1000000.0) //ITG3200   14.375 LSB/(deg/s) and we ignore the last 2 bits
#elif defined(L3G4200D)
  #define GYRO_SCALE ((4.0f * PI * 70.0f)/(1000.0f * 180.0f * 1000000.0f))
#elif defined(MPU6050)
  #define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0)   //MPU6050 and MPU3050   16.4 LSB/(deg/s) and we ignore the last 2 bits
#elif defined(LSM330)
  #define GYRO_SCALE ((4.0f * PI * 70.0f)/(1000.0f * 180.0f * 1000000.0f)) // like L3G4200D
#elif defined(L3GD20)
  #define GYRO_SCALE ((4.0f * PI * 70.0f)/(1000.0f * 180.0f * 1000000.0f)) // like L3G4200D (not sure)
#elif defined(MPU3050)
  #define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0)   //MPU6050 and MPU3050   16.4 LSB/(deg/s) and we ignore the last 2 bits
#else
  #define GYRO_SCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) //GYRO_SCALE unit: radian/microsecond
#endif

static int16_t gyroADCprevious[3] = {0,0,0};
static int16_t gyroADCp[3];
static int16_t gyroADCinter[3];

static float invG;
static int32_t accLPF32[3] = {0, 0, 1};

typedef struct fp_vector {		
  float X,Y,Z;		
} t_fp_vector_def;

typedef union {		
  float A[3];		
  t_fp_vector_def V;		
} t_fp_vector;

typedef struct int32_t_vector {
  int32_t X,Y,Z;
} t_int32_t_vector_def;

typedef union {
  int32_t A[3];
  t_int32_t_vector_def V;
} t_int32_t_vector;

static t_fp_vector EstG;
static t_int32_t_vector EstG32;

#if defined(MAGNETOMETER) && defined(useMAG)
  static t_fp_vector EstM;
  static t_int32_t_vector EstM32;
#endif

#if defined(BAROMETER) && defined(useBARO)
  #define UPDATE_INTERVAL 25000      // 40hz update rate (20hz LPF on acc)
  #define INIT_DELAY      4000000    // 4 sec initialization delay
  #define BARO_TAB_SIZE   21         //40
  #define ACC_Z_DEADBAND (ACC_1G>>5) // was 40 instead of 32 now
#endif

#if defined(THROTTLE_ANGLE_CORRECTION)
  int16_t throttleAngleCorrection = 0;	// correction of throttle in lateral wind,
  int8_t cosZ = 100;					// cos(angleZ)*100
#endif
#endif
//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                       Serial UART Constant & Variables                   //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define ISP_BUFFERING            100
#define ISP_KEEP_ALIVE           101
#define ISP_RAW_IMU              102
#define ISP_ATTITUDE             103
#define ISP_RXDATA               104
#define ISP_MOTORCMD             105
#define ISP_SET_PID              106
#define ISP_GET_PID              107
#define ISP_CPU_CYCLE            108
#define ISP_GYRO_CALIBRATION     109
#define ISP_ACC_CALIBRATION      110
#define ISP_MINMAX_CMD           111
#define ISP_GET_ALT              112
#define ISP_BARO_CALIBRATION     113
#define ISP_RX_SEND_MOUSE        114
#define ISP_MOUSE_MODE           115
#define ISP_SET_CONFIG           116
#define ISP_GET_CONFIG           117
#define ISP_KINECT_MODE          118
#define ISP_RX_SEND_KINECT       119

#define RX_BUFFER_SIZE 64
#define TX_BUFFER_SIZE 128
#define INBUF_SIZE 64

static volatile uint8_t serialHeadRX,serialTailRX;
static uint8_t serialBufferRX[RX_BUFFER_SIZE];
static volatile uint8_t serialHeadTX,serialTailTX;
static uint8_t serialBufferTX[TX_BUFFER_SIZE];
static uint8_t inBuf[INBUF_SIZE];
static uint8_t checksum;
static uint8_t indRX;
static uint8_t cmdMSP;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                         CHR_UM6 Constant & Variables                     //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

//UM^ Configuration Register
#define UM6_COMMUNICATION 0x00
#define UM6_MISC_CONFIG 0x01
#define UM6_MAG_REF_X 0x02
#define UM6_MAG_REF_Y 0x03
#define UM6_MAG_REF_Z 0x04
#define UM6_ACCEL_REF_X 0x05
#define UM6_ACCEL_REF_Y 0x06
#define UM6_ACCEL_REF_Z 0x07
#define UM6_EKF_MAG_VARIANCE 0x08
#define UM6_EKF_ACCEL_VARIANCE 0x09
#define UM6_EKF_PROCESS_VARIANCE 0x0A
#define UM6_GYRO_BIAS_XY 0x0B
#define UM6_GYRO_BIAS_Z 0x0C
#define UM6_ACCEL_BIAS_XY 0x0D
#define UM6_ACCEL_BIAS_Z 0x0E
#define UM6_MAG_BIAS_XY 0x0F
#define UM6_MAG_BIAS_Z 0x10
#define UM6_ACCEL_CAL_00 0x11
#define UM6_ACCEL_CAL_01 0x12
#define UM6_ACCEL_CAL_02 0x13
#define UM6_ACCEL_CAL_10 0x14
#define UM6_ACCEL_CAL_11 0x15
#define UM6_ACCEL_CAL_12 0x16
#define UM6_ACCEL_CAL_20 0x17
#define UM6_ACCEL_CAL_21 0x18
#define UM6_ACCEL_CAL_22 0x19
#define UM6_GYRO_CAL_00 0x1A
#define UM6_GYRO_CAL_01 0x1B
#define UM6_GYRO_CAL_02 0x1C
#define UM6_GYRO_CAL_10 0x1D
#define UM6_GYRO_CAL_11 0x1E
#define UM6_GYRO_CAL_12 0x1F
#define UM6_GYRO_CAL_20 0x20
#define UM6_GYRO_CAL_21 0x21
#define UM6_GYRO_CAL_22 0x22
#define UM6_MAG_CAL_00 0x23
#define UM6_MAG_CAL_01 0x24
#define UM6_MAG_CAL_02 0x25
#define UM6_MAG_CAL_10 0x26
#define UM6_MAG_CAL_11 0x27
#define UM6_MAG_CAL_12 0x28
#define UM6_MAG_CAL_20 0x29
#define UM6_MAG_CAL_21 0x2A
#define UM6_MAG_CAL_22 0x2B
#define UM6_GYROX_BIAS_0 0x2C
#define UM6_GYROX_BIAS_1 0x2D
#define UM6_GYROX_BIAS_2 0x2E
#define UM6_GYROX_BIAS_3 0x2F
#define UM6_GYROY_BIAS_0 0x30
#define UM6_GYROY_BIAS_1 0x31
#define UM6_GYROY_BIAS_2 0x32
#define UM6_GYROY_BIAS_3 0x33
#define UM6_GYROZ_BIAS_0 0x34
#define UM6_GYROZ_BIAS_1 0x35
#define UM6_GYROZ_BIAS_2 0x36
#define UM6_GYROZ_BIAS_3 0x37
#define UM6_GPS_HOME_LAT 0x38
#define UM6_GPS_HOME_LON 0x39
#define UM6_GPS_HOME_ALTITUDE 0x40

//UM6 Data Register
#define UM6_STATUS 0x55
#define UM6_GYRO_RAW_XY 0x56
#define UM6_GYRO_RAW_Z 0x57
#define UM6_ACCEL_RAW_XY 0x58
#define UM6_ACCEL_RAW_Z 0x59
#define UM6_MAG_RAW_XY 0x5A
#define UM6_MAG_RAW_Z 0x5B
#define UM6_GYRO_PROC_XY 0x5C
#define UM6_GYRO_PROC_Z 0x5D
#define UM6_ACCEL_PROC_XY 0x5E
#define UM6_ACCEL_PROC_Z 0x5F
#define UM6_MAG_PROC_XY 0x60
#define UM6_MAG_PROC_Z 0x61
#define UM6_EULER_PHI_THETA 0x62
#define UM6_EULER_PSI 0x63
#define UM6_QUAT_AB 0x64
#define UM6_QUAT_CD 0x65
#define UM6_ERROR_COV_00 0x66
#define UM6_ERROR_COV_01 0x67
#define UM6_ERROR_COV_02 0x68
#define UM6_ERROR_COV_03 0x69
#define UM6_ERROR_COV_10 0x6A
#define UM6_ERROR_COV_11 0x6B
#define UM6_ERROR_COV_12 0x6C
#define UM6_ERROR_COV_13 0x6D
#define UM6_ERROR_COV_20 0x6E
#define UM6_ERROR_COV_21 0x6F
#define UM6_ERROR_COV_22 0x70
#define UM6_ERROR_COV_23 0x71
#define UM6_ERROR_COV_30 0x72
#define UM6_ERROR_COV_31 0x73
#define UM6_ERROR_COV_32 0x74
#define UM6_ERROR_COV_33 0x75
#define UM6_TEMPERATURE 0x76
#define UM6_GPS_LONGITUDE 0x77
#define UM6_GPS_LATITUDE 0x78
#define UM6_GPS_ALTITUDE 0x79
#define UM6_GPS_POSITION_N 0x7A
#define UM6_GPS_POSITION_E 0x7B
#define UM6_GPS_POSITION_H 0x7C
#define UM6_GPS_COURSE_SPEED 0x7D
#define UM6_GPS_SAT_SUMMARY 0x7E
#define UM6_GPS_SAT_1_2 0x7F
#define UM6_GPS_SAT_3_4 0x80
#define UM6_GPS_SAT_5_6 0x81
#define UM6_GPS_SAT_7_8 0x82
#define UM6_GPS_SAT_9_10 0x83
#define UM6_GPS_SAT_11_12 0x84

//UM6 Command Register
#define UM6_GET_FW_VERSION 0xAA
#define UM6_FLASH_COMMIT 0xAB
#define UM6_ZERO_GYROS 0xAC
#define UM6_RESET_EKF 0xAD
#define UM6_GET_DATA 0xAE
#define UM6_SET_ACCEL_REF 0xAF
#define UM6_SET_MAG_REF 0xB0
#define UM6_RESET_TO_FACTORY 0xB1
//#define RESERVED 0xB2
#define UM6_SET_HOME_POSITION 0xB3
#define UM6_BAD_CHECKSUM 0xFD
#define UM6_UNKNOWN_ADDRESS 0xFE
#define UM6_INVALID_BATCH_SIZE 0xFF

int nState = 0;
#define STATE_ZERO         0
#define STATE_S            1
#define STATE_SN           2
#define STATE_SNP          3
#define STATE_PT           4
#define STATE_READ_DATA    5
#define STATE_CHK1         6
#define STATE_CHK0         7
#define STATE_DONE         8

#define PT_HAS_DATA  0b10000000
#define PT_IS_BATCH  0b01000000
#define PT_COMM_FAIL 0b00000001

#define DATA_BUFF_LEN  16

static volatile uint8_t serialHeadRX1,serialTailRX1;
static volatile uint8_t serialHeadTX1,serialTailTX1;
static uint8_t serialBufferRX1[DATA_BUFF_LEN];
static uint8_t serialBufferTX1[DATA_BUFF_LEN];
static uint8_t inBuf1[DATA_BUFF_LEN];
static uint8_t checksum1;
static uint8_t indRX1;
static uint8_t cmdMSP1;

byte aPacketData[DATA_BUFF_LEN];
int n = 0;
byte c = 0;
int nDataByteCount = 0;

typedef struct {
  boolean HasData;
  boolean IsBatch;
  byte BatchLength;
  boolean CommFail;
  byte Address;
  byte Checksum1;
  byte Checksum0;
  byte DataLength;
} UM6_PacketStruct ;

UM6_PacketStruct UM6_Packet;

#if defined(CHR_UM6)
  float euler[3], gyroData[3], accSmooth[3], magRAW[3];
#endif
