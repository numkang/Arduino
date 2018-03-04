static uint32_t currentTime = 0;
static uint32_t previousTime = 0;
static uint16_t cycleTime = 0;
static uint32_t taskTime1s = 0, taskTime50Hz = 0;

#define BUTTON_A B00000010
#define BUTTON_B B00001000
#define BUTTON_C B00000001
#define BUTTON_D B00000100

#define SWITCH_PIN B00001000
uint8_t pin_switch = 1;
uint8_t mission = 0;

#define RELAY_1 B10000000
#define RELAY_2 B01000000
#define RELAY_3 B00100000 //Red
#define RELAY_4 B00010000 //Green

#define PWM_HIGH 2000
#define PWM_LOW  1000

#define PWM_Pin9  0
#define PWM_Pin10 1

#define CH7_MotorInterlock PWM_Pin9
#define CH8_Brake PWM_Pin10

#define RC_CHANS 2
#define PCINT_PIN_COUNT 2
#define PCINT_RX_BITS (1<<0),(1<<3)
#define RX_PCINT_PIN_PORT PINB

#define board_trig 0
#define remote_trig 1

static int16_t rcData[RC_CHANS]; //interval [1000;2000]
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS};
volatile uint16_t rcValue[RC_CHANS];
volatile int16_t RC_Command[RC_CHANS];
