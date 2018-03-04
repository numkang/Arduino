#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define UNO
  #define PROMINI
  #define UART_NUMBER 1
#endif

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  1000 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

RTC_DS1307 RTC; // define the Real Time Clock object
DateTime now;

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;
char filename[] = "LOG_0000.txt";

/// Time ///
static uint32_t currentTime = 0;
static uint32_t previousTime = 0;
static uint32_t cycleTime = 0;
static uint32_t Interval1s = 0, Interval2s = 0, task50Hz = 0;

/// Interrupt ///
#define RC_CHANS 5
#define PCINT_PIN_COUNT 5
#define PCINT_RX_BITS (1<<2),(1<<4),(1<<5),(1<<6),(1<<7)
#define RX_PCINT_PIN_PORT PIND

static int16_t rcData[RC_CHANS]; //interval [1000;2000]
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS};
volatile uint16_t rcValue[RC_CHANS];
volatile int16_t RC_Command[RC_CHANS];

bool isTrig = false;

/// GPS ///

#define FRAME_GGA  1
#define FRAME_RMC  2

int n;
char c;

static float GPS_coord[2];
static uint8_t GPS_numSat;
static float GPS_altitude; // GPS altitude - unit: meter
//static uint16_t GPS_speed; // GPS speed - unit: cm/s
//static uint16_t GPS_ground_course = 0; // - unit: degree*10
static uint8_t GPS_Present = 0; // Checksum from Gps serial
static uint8_t GPS_FIX = 0;
//static int32_t GPS_payload_GGA = 0;
//static int32_t GPS_payload_RMC = 0;
//static float GPS_ecef[3];
//static char gps_serial_GGA[128];
//static char gps_serial_RMC[128];
//static float ground_speed;

#define LAT  0
#define LON  1
 
//WGS84 ellipsoid constants:
int32_t a = 6378137;
float e = 8.1819190842622e-2;
  
uint8_t frameOK = 0;
static uint8_t param = 0, offset = 0, offset2 = 0, parity = 0; //parity_GGA = 0, parity_RMC = 0;
static char string[15];
//static char string2[256];
static uint8_t checksum_param, frame = 0, stop_GGA = 0, stop_RMC = 0;
  
char time[9];
//char lat[] = {"0000.00000"};
//char lon[] = {"00000.00000"};
char checksum[2];
//char gpsSpeed[] = {"2.297"};
uint8_t isSouth = 0;
uint8_t isWest = 0;
