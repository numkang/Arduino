
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

#define UM6_GET_DATA       0xAE
#define UM6_QUAT_AB        0x64
#define UM6_QUAT_CD        0x65
#define UM6_GYRO_PROC_XY   0x5C
#define UM6_GYRO_PROC_Z    0x5D
#define UM6_ACCEL_PROC_XY  0x5E
#define UM6_ACCEL_PROC_Z   0x5F
#define UM6_EULER_PHI_THETA 0x62
#define UM6_EULER_PSI 0x63

#define QUAT_SCALE_FACTOR  0.0000335693   // Convert to Quaternions
#define ACCEL_SCALE_FACTOR 0.000183105    // Convert to Gravity
#define GYRO_SCALE_FACTOR  0.0610352      // Convert to Degrees per Second

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

float DataA = 0;
float DataB = 0;
float DataC = 0;
float DataD = 0;
