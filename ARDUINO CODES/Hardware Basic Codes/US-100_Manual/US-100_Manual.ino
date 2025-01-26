#define TRIGGER A4
#define ECHO    A5
 
void setup() {
 
  Serial.begin (9600);
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  // pinMode(BUILTIN_LED, OUTPUT);
}
 
void loop() {
 
  long duration;
  long distance;
  digitalWrite(TRIGGER, LOW);  
  delayMicroseconds(2); 
  
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10); 
  
  digitalWrite(TRIGGER, LOW);
  duration = pulseIn(ECHO, HIGH);
  distance = duration * 0.034 / 2;
  
  if (distance >= 400 ) {
      Serial.println("Out of range");
  } else {
      Serial.print("Centimeter: ");
      Serial.println(distance);
  }
}