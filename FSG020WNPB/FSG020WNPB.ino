void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int pin2 = analogRead(A2);
  int pin4 = analogRead(A4);
  float volt2 = pin2 * 5.0 / 1023.0;
  float volt4 = pin4 * 5.0 / 1023.0;
  float dVolt = volt2 - volt4;
  Serial.print(pin2);
  Serial.print(" , ");
  Serial.print(pin4);
  Serial.print(" , ");
  Serial.print(volt2);
  Serial.print(" , ");
  Serial.print(volt4);
  Serial.print(" , ");
  Serial.println(dVolt);
  delay(100);
}
