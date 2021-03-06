//Making Burglar alarm with Ultrasonic and Buzzer 

int echoPin = 11;
int triggerPin = 12;
int buzzerPin = 9;

int maximumRange = 400; //cm
int minimumRange = 0; //cm
int BuzzerRange = 100; //cm
long ultrasonicDuration, distance;
int buzzerDuration = 150; //To make alarm faster, decrease this value
int buzzerFrequency = 300;

void setup() 
{
 Serial.begin (57600);
 pinMode(triggerPin, OUTPUT);
 pinMode(echoPin, INPUT);
 pinMode(buzzerPin, OUTPUT);
}

void loop() 
{ 
 digitalWrite(triggerPin, LOW); 
 delayMicroseconds(2); 

 digitalWrite(triggerPin, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(triggerPin, LOW);
 ultrasonicDuration = pulseIn(echoPin, HIGH);
 
 distance = ultrasonicDuration/58.2; //cm
 
 if (distance <= BuzzerRange && distance !=  0)
 {
   tone(buzzerPin, buzzerFrequency, buzzerDuration);
   delay(buzzerDuration);
 }
 
 if (distance >= maximumRange || distance <= minimumRange){
 Serial.println("out of range"); //ultrasonic read zero
 }
 else {
 Serial.println(distance);
 }
 delay(50);
}
