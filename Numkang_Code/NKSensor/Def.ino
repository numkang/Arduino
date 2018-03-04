//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                              AVR Microchips Setup                        //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define UNO
  #define PROMINI
#endif

#if defined(__AVR_ATmega32U4__)
  #define NANO
#endif

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define MEGA
#endif

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                           General function constants                     //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#define THR 0

enum rc {
  ROLL,
  PITCH,
  YAW
};

struct flags_struct {
  uint8_t gyroCalibrated :1 ;
  uint8_t accCalibrated :1 ;
  uint8_t magCalibrated :1 ;
  uint8_t motorCalibrated :1 ;
  uint8_t magSetHeading :1 ;
  uint8_t SMALL_ANGLES_25 :1 ;
} f;

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                      Control Board Constant & Variables                  //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

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
  #define MPU6050
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

#if defined(CRIUS_SE_v2)
  #define MPU6050
  #define HMC5883
  #define BMP085
  #define ACC_ORIENTATION(X, Y, Z)  {accRAW[ROLL]  = -X; accRAW[PITCH]  = -Y; accRAW[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroRAW[ROLL] =  Y; gyroRAW[PITCH] = -X; gyroRAW[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magRAW[ROLL]  =  X; magRAW[PITCH]  =  Y; magRAW[YAW]  = -Z;}
  #define MPU6050_I2C_AUX_MASTER
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

#if defined(AltIMU)
  #define L3GD20
  #define LSM303DLHC //Acc+Mag
  #define LPS331AP
  #define ACC_ORIENTATION(X, Y, Z)  {accRAW[ROLL]  =  X; accRAW[PITCH]  =  Y; accRAW[YAW]  =  Z;}
  #define GYRO_ORIENTATION(X, Y, Z) {gyroRAW[ROLL] =  Y; gyroRAW[PITCH] = -X; gyroRAW[YAW] = -Z;}
  #define MAG_ORIENTATION(X, Y, Z)  {magRAW[ROLL]  =  X; magRAW[PITCH]  =  Y; magRAW[YAW]  =  Z;}
#endif

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                      Sensor Chip Constant & Variables                    //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////

#if defined(ADXL345) || defined(BMA180) || defined(MMA7361) || defined(MPU6050) || defined(LSM303DLHC)
  #define ACCELEROMETER
#endif

#if defined(L3G4200D) || defined(ITG3205) || defined(MPU6050) || defined(L3GD20)
  #define GYROSCOPE
#endif

#if defined(HMC5883) || defined(AK8975) || defined(LSM303DLHC) && defined(useMAG)
  #define MAGNETOMETER
#endif

#if defined(BMP085) || defined(MS561101BA) || defined(LPS331AP) && defined(useBARO)
  #define BAROMETER
#endif

#if defined(HCSR04) && defined(useSONAR)
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
  #define MPU6050_DLPF_CFG 0 //Default settings LPF 256Hz/8000Hz sample
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
  #define ITG3205_SMPLRT_DIV 0 //Default settings LPF 256Hz/8000Hz sample
  #define ITG3205_DLPF_CFG   0
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

//I2C
uint8_t rawADC[6];
static uint32_t neutralizeTime = 0;
static int16_t i2c_errors_count = 0;

#if defined(ACCELEROMETER)
  #define ACC_1G 265
  #define ACC_TO_SAMPLE 400
  static uint16_t acc_1G = 265;
  static uint16_t acc_25deg;
  static uint16_t ACC_SAMPLE = ACC_TO_SAMPLE;
  static int16_t accRAW[3], accSmooth[3];
  static struct {int16_t zero_offset[3];} ACCEL;
#endif

#if defined(GYROSCOPE)
  #define GYRO_TO_SAMPLE 400
  static int16_t gyroRAW[3];
  static uint16_t GYRO_SAMPLE = GYRO_TO_SAMPLE;
  static struct {int16_t zero_offset[3];} GYRO;
  static int16_t gyroData[3] = {0,0,0};
#endif

#if defined(MAGNETOMETER)
  static int16_t magRAW[3], magSmooth[3];
  static float magScale[3] = {1.0,1.0,1.0};
  static uint8_t magInit = 0;
  static struct {int16_t zero_offset[3];} MAG;
#endif
  
#if defined(BAROMETER)
  static int32_t baroPressure;
  static int32_t baroTemperature;
  static int32_t baroPressureSum;
  static uint16_t calibratingB = 0;  // baro calibration = get new ground pressure value
  static int32_t BaroAlt,EstAlt,AltHold; // in cm
  static int16_t BaroPID = 0;
  static int16_t errorAltitudeI = 0;
  static int16_t vario = 0;
  #if defined(LPS331AP)  
    uint8_t UP_RAW[3];
    uint8_t UT_RAW[2];
    uint32_t LPS331AP_deadline;
  #endif
#endif

#if defined(ULTRASONIC)
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

#define ACC_LPF_FACTOR 100
#define GYR_CMPFA_FACTOR 400.0f //600
#define GYR_CMPFM_FACTOR 200.0f //250,300
#define INV_GYR_CMPFA_FACTOR (1.0f / (GYR_CMPFA_FACTOR + 1.0f))
#define INV_GYR_CMPFM_FACTOR (1.0f / (GYR_CMPFM_FACTOR + 1.0f))

#define GYRO_SCALE ((2380 * PI)/((32767.0f / 4.0f ) * 180.0f * 1000000.0f)) // GYRO_SCALE unit: radian/microsecond
#define RAW_TO_DEG_FACTOR 10
#define MAG_DECLINIATION 0.0f

static int16_t angle[3] = {0,0,0};
static int16_t heading;
static float invG; // 1/|G|
static int32_t accLPF32[3] = {0, 0, 1};

typedef struct fp_vector {float X; float Y; float Z;} t_fp_vector_def;
typedef union {float A[3]; t_fp_vector_def V;} t_fp_vector;

#define UPDATE_INTERVAL 25000    // 40hz update rate (20hz LPF on acc)
#define INIT_DELAY      4000000  // 4 sec initialization delay
#define BARO_TAB_SIZE   21       //40
#define ACC_Z_DEADBAND (acc_1G>>5) // was 40 instead of 32 now
