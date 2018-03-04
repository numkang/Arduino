//Arduino 1.0+ only

#include <Wire.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; //I2C address of the L3G4200D

int x = 0;
float p_z = 0;
int y;
int z = 0;
float degreesPerSecond = 0;
float gyroRate = 0;
float currentAngle = 0;
float currentAngleDegrees = 0;
long currMillis = 0;
long pastMillis = 0;
long calibrationSum = 0;
int gyroZero = 0;
int gyroHigh = 0;
int gyroLow = 0;
unsigned long currMicros = 0;
unsigned long pastMicros = 0;
float dt = 0;

void setup(){

  Wire.begin();
  Serial.begin(115200);

  Serial.println("starting up L3G4200D");
  setupL3G4200D(); // Configure L3G4200  - 250, 500 or 2000 deg/sec

  delay(1500); //wait for the sensor to be ready 
  calibrate();
}

void loop()
{
  pastMillis = millis();
  getGyroValues();
  
  pastMicros = currMicros;
  currMicros = micros();
  
  if(z >= gyroHigh || z <= gyroLow)
  {
    degreesPerSecond = (float)z * 0.00000908; //this value was 0.0000085 but was manually calibrated to its current value
    if (currMicros>pastMicros)  //micros() overflows/resets every ~70 minutes
      dt = (float) (currMicros-pastMicros)/1000000.0;
    else
      dt = (float) ((4294967295-pastMicros)+currMicros)/1000000.0;
    currentAngle += ((p_z + degreesPerSecond)/2) * dt;
    p_z = degreesPerSecond;
  }
  else 
  {
    p_z = 0;
  }
  Serial.println(currentAngle * 1000);
  delay(10);
}

void calibrate()
{
  for(int i = 0; i < 400; i++)
  {
    getGyroValues();

    if(z > gyroHigh)
    {
      gyroHigh = z;
    }
    else if(z < gyroLow)
    {
      gyroLow = z;
    }
  }
}

void getGyroValues(){

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  y = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB);
}

int setupL3G4200D(){
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b10001111);
  writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}


