#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;


void setup()
{

  Serial.begin(9600);
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();
}

void loop()
{
  int distance =sensor.readRangeContinuousMillimeters();
  //int distance =sensor.startContinuous(100);
  
 //distance = distance;
  Serial.print("Distance: ");
  Serial.print(distance-8);
  Serial.print("mm");
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.println();
  delay(100);
}

 
