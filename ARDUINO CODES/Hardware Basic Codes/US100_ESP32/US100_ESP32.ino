#define TRIGGER 18  // GPIO pin for TRIGGER
#define ECHO    19  // GPIO pin for ECHO

void setup() {

  Serial.begin(115200);
  pinMode(TRIGGER, OUTPUT);
  pinMode(ECHO, INPUT);
  // pinMode(BUILTIN_LED, OUTPUT);  // Uncomment if using the built-in LED

}

void loop() {

  long duration;
  long distance;

  // Trigger the ultrasonic sensor
  digitalWrite(TRIGGER, LOW);  
  delayMicroseconds(2); 
  
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10); 
  
  digitalWrite(TRIGGER, LOW);

  // Measure the duration of the echo pulse
  duration = pulseInLong(ECHO, HIGH);

  // Calculate the distance in centimeters
  distance = duration * 0.034 / 2;

  // Output the distance or out-of-range message
  if (distance >= 400) {
    Serial.println("Out of range");
  } else {
    Serial.print("Centimeter: ");
    Serial.println(distance);
  }

  delay(500);  // Wait before the next measurement
}

