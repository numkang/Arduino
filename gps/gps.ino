#define FRAME_GGA  1
#define FRAME_RMC  2

int n;
char c;

static uint32_t currentTime = 0;
static uint32_t previousTime = 0;
static uint16_t cycleTime = 0;
static uint16_t cycleTime2 = 0;
static uint32_t taskTime = 0;
static uint32_t taskTime2 = 0;
  
// **********************
// GPS common variables
// **********************
  static float  GPS_coord[2];
  static uint8_t  GPS_numSat;
  static float  GPS_altitude;                                // GPS altitude      - unit: meter
  static uint16_t GPS_speed;                                   // GPS speed         - unit: cm/s
  static uint16_t GPS_ground_course = 0;                       //                   - unit: degree*10
  static uint8_t  GPS_Present = 0;                             // Checksum from Gps serial
  static uint8_t  GPS_FIX  = 0;
  static int32_t  GPS_payload_GGA = 0;
  static int32_t  GPS_payload_RMC = 0;
  static float  GPS_ecef[3];
  static char gps_serial_GGA[128];
  static char gps_serial_RMC[128];
  static float ground_speed;

  #define LAT  0
  #define LON  1
  #define X 0
  #define Y 1
  #define Z 2
  
  //WGS84 ellipsoid constants:
  int32_t a = 6378137;
  float e = 8.1819190842622e-2;
  
  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, offset2 = 0, parity = 0, parity_GGA = 0, parity_RMC = 0;
  static char string[15];
  static char string2[256];
  static uint8_t checksum_param, frame = 0, stop_GGA = 0, stop_RMC = 0;
  
  char time[9];
  char lat[] = {"0000.00000"};
  char lon[] = {"00000.00000"};
  char checksum[2];
  char gpsSpeed[] = {"2.297"};
  uint8_t isSouth = 0;
  uint8_t isWest = 0;
