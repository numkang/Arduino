char a[] ={"GPRMC,090447.00,A,1350.71617,N,10034.22353,E,0.150,,230414,,,D"}; //
uint8_t x = 0;
int l;

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
l = sizeof(a);
for(int i=0; i<l; i++){
    x ^= a[i]; 
  }
  Serial.println(x,HEX);
}


void loop() {
  // put your main code here, to run repeatedly:
}
