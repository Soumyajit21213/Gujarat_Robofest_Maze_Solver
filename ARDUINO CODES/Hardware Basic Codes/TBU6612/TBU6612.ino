// const int AI1 = 4 ;
// const int AI2 = 5 ;
// const int PWM_A = 6 ;

const int BI1 = 6 ;
const int BI2 = 7 ;
// const int PWM_B = 10 ;

// int speed = 255;

void setup() {
  // put your setup code here, to run once:
// pinMode(AI1, OUTPUT);
// pinMode(AI2, OUTPUT);
// pinMode(PWM_A, OUTPUT);
pinMode(BI1, OUTPUT);
pinMode(BI2, OUTPUT);
// pinMode(PWM_B, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
// digitalWrite(AI1,LOW);
// digitalWrite(AI2,HIGH);
// analogWrite(PWM_A,speed);
// digitalWrite(BI1,LOW);
// digitalWrite(BI2,HIGH);
// analogWrite(PWM_B,speed);

digitalWrite(BI1,LOW);
digitalWrite(BI2,HIGH);
delay(2000);
digitalWrite(BI1,HIGH);
digitalWrite(BI2,LOW);
delay(2000);
}
