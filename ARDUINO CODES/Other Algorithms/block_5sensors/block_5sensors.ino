int in1 = 16;
int in2 = 17;
int in3 = 8;
int in4 = 3;
int enA = 15;
int enB = 46;
int speeda = 150;
int speedb = 150;
int stdby = 18;

const int encoderA1 = 5;
const int encoderA2 = 6;
const int encoderB1 = 12;
const int encoderB2 = 11;

float unitLength = 23;
int width = 12;
int length =14;
long aposi = 0;
long bposi = 0;

int trigpinF = 40;
int echopinF = 37;
int trigpinL = 42;
int echopinL = 41;
int trigpinR = 36;
int echopinR = 35;
const int   trigPinLF=39,echoPinLF=19, trigPinRF=43,echoPinRF=20;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(921600);  //Set the baud rate to your Bluetooth module.
  pinMode(trigpinF,OUTPUT);
  pinMode(echopinF,INPUT);
  pinMode(trigpinL,OUTPUT);
  pinMode(echopinL,INPUT);
  pinMode(trigpinR,OUTPUT);
  pinMode(echopinR,INPUT);
  //pinMode(trigPinRF, OUTPUT); pinMode(echoPinRF, INPUT);
  //pinMode(trigPinLF, OUTPUT); pinMode(echoPinLF, INPUT);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(stdby,OUTPUT);

  pinMode(encoderA1, INPUT); // Set pin mode for encoderA1 (encoder A)
  pinMode(encoderA2, INPUT); // Set pin mode for encoderA2 (encoder B)
  pinMode(encoderB1, INPUT); // Set pin mode for encoderA1 (encoder A)
  pinMode(encoderB2, INPUT); // Set pin mode for encoderA2 (encoder B)
  attachInterrupt(digitalPinToInterrupt(encoderA1), handleEncoderA, RISING); 
  attachInterrupt(digitalPinToInterrupt(encoderB1), handleEncoderB, RISING);
 
}

void loop() {
  // put your main code here, to run repeatedly:
 
  long durationF;int distanceF;
  digitalWrite(trigpinF,HIGH);
  delay(1);
  digitalWrite(trigpinF,LOW);
  durationF=pulseIn(echopinF,HIGH);
  distanceF=(durationF/2)/29.1;

  long durationL;int distanceL;
  digitalWrite(trigpinL,HIGH);
  delay(1);
  digitalWrite(trigpinL,LOW);
  durationL=pulseIn(echopinL,HIGH);
  distanceL=(durationL/2)/29.1;

  long durationR;int distanceR;
  digitalWrite(trigpinR,HIGH);
  delay(1);
  digitalWrite(trigpinR,LOW);
  durationR=pulseIn(echopinR,HIGH);
  distanceR=(durationR/2)/29.1;

  /*long durationLF;int distanceLF;
  digitalWrite(trigPinLF,HIGH);
  delay(1);
  digitalWrite(trigPinLF,LOW);
  durationLF=pulseIn(echoPinLF,HIGH);
  distanceLF=(durationLF/2)/29.1;

  long durationRF;int distanceRF;
  digitalWrite(trigPinRF,HIGH);
  delay(1);
  digitalWrite(trigPinRF,LOW);
  durationRF=pulseIn(echoPinRF,HIGH);
  distanceRF=(durationRF/2)/29.1;*/


if((distanceR<=(unitLength-width)/2 && distanceF>=unitLength && distanceL<=unitLength-width)||(distanceL>=unitLength && distanceR<=unitLength-width&& distanceF<=unitLength-length)||(distanceL<=(unitLength-width)/2 && distanceF>=unitLength && distanceR<=unitLength-width)||(distanceR>=unitLength && distanceL<=unitLength-width&& distanceF<=unitLength-length)||(distanceR>=unitLength&&distanceL>=unitLength)||(distanceL>=unitLength&&distanceF>=unitLength)||(distanceR>=unitLength&&distanceF>=unitLength)||distanceL>=300||distanceR>=300){

  long apos = aposi ;
  long bpos = bposi ;

  if ((distanceR<=(unitLength-width)/2&& distanceF>=unitLength && distanceL<=unitLength-width)||distanceR>=300)  { left();delay(20);stop();
                                    }
  else if (distanceL>=unitLength && distanceR<=unitLength-(width/2) +3&& distanceF<=unitLength-length-2)  {  
                                       left();
                                      delay(400);
                                      forward();
                                      delay(800);
                                      Serial.print(" L ");
                                    }
  if ((distanceL<=(unitLength-width)/2 && distanceF>=unitLength && distanceR<=unitLength-width)||distanceL>=300)  { right();delay(20);stop();
                                    }
  else if (distanceR>=unitLength && distanceL<=unitLength-(width/2)+3&& distanceF<=unitLength-length-2)  {  
                                      right();
                                      delay(400);
                                      forward();
                                      delay(800);
                                      Serial.print(" R ");
                                    }     
  if((distanceR>=unitLength&&distanceL>=unitLength)||(distanceL>=unitLength&&distanceF>=unitLength)||(distanceR>=unitLength&&distanceF>=unitLength)){
          int tempL=distanceL,tempR=distanceR,tempF=distanceF;
           if(distanceL>distanceR){left();delay(80);stop();}
           else{right();delay(80);stop();}
           digitalWrite(trigpinF,HIGH);
           delay(1);
     digitalWrite(trigpinF,LOW);
  durationF=pulseIn(echopinF,HIGH);
  distanceF=(durationF/2)/29.1;

  
  digitalWrite(trigpinL,HIGH);
  delay(1);
  digitalWrite(trigpinL,LOW);
  durationL=pulseIn(echopinL,HIGH);
  distanceL=(durationL/2)/29.1;

  
  digitalWrite(trigpinR,HIGH);
  delay(1);
  digitalWrite(trigpinR,LOW);
  durationR=pulseIn(echopinR,HIGH);
  distanceR=(durationR/2)/29.1;

  if((distanceR>=unitLength&&distanceL>=unitLength)||(distanceL>=unitLength&&distanceF>=unitLength)||(distanceR>=unitLength&&distanceF>=unitLength)){stop();delay(2000);}

 }      */                                                                           
  
}}
else{forward();}



}
void backward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(stdby, HIGH);
  analogWrite(enA,speeda);
  analogWrite(enB,speedb);

}
void forward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(stdby, HIGH);
  analogWrite(enA,speeda);
  analogWrite(enB,speedb);

}
void right() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(stdby, HIGH);
  analogWrite(enA,speeda);
  analogWrite(enB,speedb);

}
void left() { 
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(stdby, HIGH);
  analogWrite(enA,speeda);
  analogWrite(enB,speedb);

}

void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(stdby, LOW);
  Serial.println(" stop");
}

void handleEncoderA() {
  int a = digitalRead(encoderA2); // Read the state of encoderA2 (encoder B)
  if (a > 0) {
    aposi++; // Increment position if encoderA2 is HIGH
  } else {
    aposi--; // Decrement position if encoderA2 is LOW
  }
}
void handleEncoderB() {
  int b = digitalRead(encoderB2); // Read the state of encoderA2 (encoder B)
  if (b > 0) {
    bposi++; // Increment position if encoderA2 is HIGH
  } else {
    bposi--; // Decrement position if encoderA2 is LOW
  }
}
