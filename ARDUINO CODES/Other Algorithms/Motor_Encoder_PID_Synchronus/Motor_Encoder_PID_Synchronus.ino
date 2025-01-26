const int encoderA1 = 5;
const int encoderA2 = 6;
const int encoderB1 = 12;
const int encoderB2 = 11;

const int AI2 = 16;
const int AI1 = 17;
const int Apwm = 15;
const int BI2 = 8;
const int BI1 = 3;
const int Bpwm = 46;
int stdby = 18;

long aposi = 0;
long bposi = 0;
float unitLength = 22;
int width = 12;
int length =14; // 1 feet = 30.48 cm 
float wheelCircumference = 14.5; // circumference = 2*pie*2.2 cm
int block = 374 * unitLength/wheelCircumference;
int speeda = 150;
int speedb = 150;

long previousTimeA = 0;
float ePreviousA = 0;
long previousTimeB = 0;
float ePreviousB = 0;
float kpa = 2.0;
float kda = 0.1;
float kpb = 2.0;
float kdb = 0.1;

int trigpinF = 40;
int echopinF = 37;
int trigpinL = 42;
int echopinL = 41;
int trigpinR = 36;
int echopinR = 35;

void setup() {
  Serial.begin(9600);

  pinMode(trigpinF,OUTPUT);
  pinMode(echopinF,INPUT);
  pinMode(trigpinL,OUTPUT);
  pinMode(echopinL,INPUT);
  pinMode(trigpinR,OUTPUT);
  pinMode(echopinR,INPUT);

  pinMode(stdby,OUTPUT);
  pinMode(AI1, OUTPUT);
  pinMode(AI2, OUTPUT);
  pinMode(Apwm,OUTPUT);
  pinMode(BI1, OUTPUT);
  pinMode(BI2, OUTPUT);
  pinMode(Bpwm,OUTPUT);
  pinMode(encoderA1, INPUT); // Set pin mode for encoderA1 (encoder A)
  pinMode(encoderA2, INPUT); // Set pin mode for encoderA2 (encoder B)
  pinMode(encoderB1, INPUT); // Set pin mode for encoderA1 (encoder A)
  pinMode(encoderB2, INPUT); // Set pin mode for encoderA2 (encoder B)
  attachInterrupt(digitalPinToInterrupt(encoderA1), handleEncoderA, RISING); 
  attachInterrupt(digitalPinToInterrupt(encoderB1), handleEncoderB, RISING);
}

void loop() {
  
Left(90);
stop();
delay(5000);
  

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
void Forward(int length){
  digitalWrite(stdby,HIGH);
  int tolerance = 20 ;
  int atarget = aposi + length  ;
  int btarget = bposi + length;
  int aError = atarget - aposi;
  int bError = btarget - bposi;
  float aSpeed;
  float bSpeed;

  while(abs(aError) > tolerance || abs(bError) > tolerance) {
    aSpeed = pidControllerA(aError);
    bSpeed = pidControllerB(bError);
    digitalWrite(AI1, HIGH);
    digitalWrite(AI2, LOW);
    analogWrite(Apwm, constrain((int)fabs(aSpeed), 0, 180));
    digitalWrite(BI1, HIGH);
    digitalWrite(BI2, LOW);
    analogWrite(Bpwm, constrain((int)fabs(bSpeed), 0, 180));
    aError = atarget - aposi ;
    bError = btarget - bposi ;
  }
  stop();
}
void Backward(int length){
  digitalWrite(stdby,HIGH);
  int tolerance = 20 ;
  int atarget = aposi - length;
  int btarget = bposi - length;
  int aError = atarget - aposi;
  int bError = btarget - bposi;
  float aSpeed;
  float bSpeed;

  while(abs(aError) > tolerance || abs(bError) > tolerance) {
    aSpeed = pidControllerA(aError);
    bSpeed = pidControllerB(bError);
    digitalWrite(AI1, LOW);
    digitalWrite(AI2, HIGH);
    analogWrite(Apwm, constrain((int)fabs(aSpeed), 0, 180));
    digitalWrite(BI1, LOW);
    digitalWrite(BI2, HIGH);
    analogWrite(Bpwm, constrain((int)fabs(bSpeed), 0, 180));
    aError = atarget - aposi;
    bError = btarget - bposi ;
  }
  stop();
}
void Left(int angle) {
  digitalWrite(stdby, HIGH); // Enable the motor driver
  int tolerance = 20;

  float turningDistance = (PI * 9.4 * angle) / 360.0; 
  int rotationDistance = 374 * turningDistance / wheelCircumference ;
  int atarget = aposi - rotationDistance; // Left wheel backward
  int btarget = bposi + rotationDistance; // Right wheel forward
  int aError = atarget - aposi;
  int bError = btarget - bposi;
  float aSpeed;
  float bSpeed;

  while (abs(aError) > tolerance || abs(bError) > tolerance) {
    aSpeed = pidControllerA(aError);
    bSpeed = pidControllerB(bError);
    digitalWrite(AI1, LOW);
    digitalWrite(AI2, HIGH);
    analogWrite(Apwm, constrain((int)fabs(aSpeed), 0, 150));
    digitalWrite(BI1, HIGH);
    digitalWrite(BI2, LOW);
    analogWrite(Bpwm, constrain((int)fabs(bSpeed), 0, 150));
    aError = atarget - aposi;
    bError = btarget - bposi;
  }
  stop();
}

void Right(int angle) {
  digitalWrite(stdby, HIGH); // Enable the motor driver
  int tolerance = 20; 
  float turningDistance = (PI * 9.4 * angle) / 360.0; 
  int rotationDistance = 374 * turningDistance / wheelCircumference ;

  int atarget = aposi + rotationDistance; // Left wheel forward
  int btarget = bposi - rotationDistance; // Right wheel backward
  int aError = atarget - aposi;
  int bError = btarget - bposi;
  float aSpeed;
  float bSpeed;

  while (abs(aError) > tolerance || abs(bError) > tolerance) {
    aSpeed = pidControllerA(aError);
    bSpeed = pidControllerB(bError);
    digitalWrite(AI1, HIGH);
    digitalWrite(AI2, LOW);
    analogWrite(Apwm, constrain((int)fabs(aSpeed), 0, 180));
    digitalWrite(BI1, LOW);
    digitalWrite(BI2, HIGH);
    analogWrite(Bpwm, constrain((int)fabs(bSpeed), 0, 180));                     
    aError = atarget - aposi;
    bError = btarget - bposi;
  }
  stop();
}

void stop(){
  digitalWrite(stdby,LOW);
}
void right() {
  digitalWrite(AI2, LOW);
  digitalWrite(AI1, HIGH);
  digitalWrite(BI2, HIGH);
  digitalWrite(BI1, LOW);
  digitalWrite(stdby, HIGH);
  analogWrite(Apwm,speeda);
  analogWrite(Bpwm,speedb);

}
void left() { 
  digitalWrite(AI2, HIGH);
  digitalWrite(AI1, LOW);
  digitalWrite(BI2, LOW);
  digitalWrite(BI1, HIGH);
  digitalWrite(stdby, HIGH);
  analogWrite(Apwm,speeda);
  analogWrite(Bpwm,speedb);
}

float pidControllerA(int e) {
  long currentTimeA = micros();
  float deltaTA = ((float)(currentTimeA - previousTimeA)) / 1.0e6;
  float eDerivativeA = (e - ePreviousA) / deltaTA;
  float u = (kpa * e) + (kda * eDerivativeA) ;
  previousTimeA = currentTimeA;
  ePreviousA = e;
  return u;
}
float pidControllerB(int e) {
  long currentTimeB = micros();
  float deltaTB = ((float)(currentTimeB - previousTimeB)) / 1.0e6;
  float eDerivativeB = (e - ePreviousB) / deltaTB;
  float u = (kpb * e) + (kdb * eDerivativeB) ;
  previousTimeB = currentTimeB;
  ePreviousB = e;
  return u;
}
