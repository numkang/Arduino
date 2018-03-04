float KP = 5.0f; //5
float KI = 5.0f;
float KD = 3.0f; //0.7

float KP_VDO = 5.0f;
float KI_VDO = 0.0f;
float KD_VDO = 0.7f;

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define UNO
  #define PROMINI
  #define NUMBER_MOTORS 2
  #define UART_NUMBER 1
#endif

#define VER1 1
#define VER2 0
#define VER3 100

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
  uint8_t c; 
  uint8_t vdo_active;
  uint8_t x;
} f;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                   LED function                           //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
#define ON_LEDPIN_13 digitalWrite(13,HIGH); //red
#define OFF_LEDPIN_13 digitalWrite(13,LOW);
#define TOGGLE_LEDPIN_13 digitalWrite(13,!digitalRead(13));
#define BLINK_LEDPIN_13(T1,T2) for(uint8_t t = 0;t < T1;t++){TOGGLE_LEDPIN_13 delay(T2);}

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                 Time function                            //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

static uint32_t currentTime = 0;
static uint32_t previousTime = 0;
static uint32_t cycleTime = 0;
static uint32_t taskTime50 = 0, taskTime400 = 0, taskTime500 = 0;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                     Serial                               //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define RX_BUFFER_SIZE 256 //256 if GPS, 64 if not
#define TX_BUFFER_SIZE 128
#define INBUF_SIZE 64

static volatile uint8_t serialHeadRX[UART_NUMBER],serialTailRX[UART_NUMBER];
static uint8_t serialBufferRX[RX_BUFFER_SIZE][UART_NUMBER];
static volatile uint8_t serialHeadTX[UART_NUMBER],serialTailTX[UART_NUMBER];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_NUMBER];
static uint8_t inBuf[INBUF_SIZE][UART_NUMBER];

static uint8_t checksum[UART_NUMBER];
static uint8_t indRX[UART_NUMBER];
static uint8_t cmdISC[UART_NUMBER];

#define ISC_BUFFERING 100
#define ISC_KEEP_ALIVE 101
#define ISC_START_STOP 102
#define ISC_IMU 103
#define ISC_VDO_ANGLE 104
#define ISC_VDO_USE 105

static float gain = 0;
static char kgain = 0;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                    Sensor                                //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define I2C_SPEED 400000L
#define ACC_ORIENTATION(X, Y, Z)  {accRAW[ROLL]  = -X; accRAW[PITCH]  = -Y; accRAW[YAW]  =  Z;}
#define GYRO_ORIENTATION(X, Y, Z) {gyroRAW[ROLL] =  Y; gyroRAW[PITCH] = -X; gyroRAW[YAW] = -Z;}

#define MPU6050_ADDRESS 0x68  
#define MPU6050_DLPF_CFG 2 //0-256 1-188 2-98 3-42 4-20 5-10 6-5

uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;
static int16_t i2c_errors_count = 0;

#define ACC_TO_SAMPLE 400
static uint16_t calibratingA = 400;
static int16_t accRAW[3], accSmooth[3];
static int16_t ACC_Zero[3];

#define GYRO_TO_SAMPLE 400
static int16_t gyroRAW[3];
static uint16_t calibratingG = 400;
static int16_t GYRO_Zero[3];
static int16_t gyroData[3] = {0,0,0};

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                  Complementary Filter Constant & Variables               //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

//static int16_t angle[3] = {0,0,0};
float angle[3] = {0,0,0};
#define ACC_LPF_FACTOR 4 //100
#define ACC_1G 512
#define ACCZ_25deg   (int16_t)(ACC_1G * 0.90631) // 0.90631 = cos(25deg) (cos(theta) of accZ comparison)
#define ACC_VelScale (9.80665f / 10000.0f / ACC_1G)

#define GYR_CMPFA_FACTOR 200.0f //400
#define GYR_CMPFM_FACTOR 250.0f //200,300
#define INV_GYR_CMPFA_FACTOR (1.0f / (GYR_CMPFA_FACTOR + 1.0f))
#define INV_GYR_CMPFM_FACTOR (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0)   //MPU6050 and MPU3050   16.4 LSB/(deg/s) and we ignore the last 2 bits

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

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                       PWM & PID Constant & Variables                     //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define MINCOMMAND 900
#define MAXCOMMAND 2200
#define PID_MIX(X,Y,Z) 1300 + X*PID_CMD[ROLL] + Y*PID_CMD[PITCH] + Z*PID_CMD[YAW];

static uint16_t PWM_Command[9];

volatile float setpoint = 0;
float PID_CMD[3];
float error, errorI, errorD, previous_error = 0;
float Pterm, Iterm, Dterm;

float vdo_angle, vdo_angle_temp, vdo_angle_previous;
uint8_t vdo_angle_check;
volatile float setpoint_VDO = 0;
float PID_CMD_VDO[3];
float error_VDO, errorI_VDO, errorD_VDO, previous_error_VDO = 0;
float Pterm_VDO, Iterm_VDO, Dterm_VDO;
float gyro_D;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                     Vector                               //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

float gyroVectorX = 0;
float gyroVectorY = 0;
float gyroVectorZ = 1;
float Gyro_AngX;
float Gyro_AngY;
float Gyro_AngZ;

float accR;
float inv_accR;
float n_accX;
float n_accY;
float n_accZ;

float trueVectorX = 0;
float trueVectorY = 0;
float trueVectorZ = 1;
float trueVectorR = 1;

float ComplementAlpha = 0.9;

float trueAng[2];

float gyroLast[3];
float dPS[3];
float gyroAngle[3];
int gyroHigh[3];
int gyroLow[3];
float dt = 0;
