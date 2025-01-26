int in1 = 16;
int in2 = 17;
int in3 = 8;
int in4 = 3;
int enA = 15;
int enB = 46;
int speeda = 150;
int speedb = 150;
int stdby = 18;

int trigpinF = 40;
int echopinF = 37;

int trigpinL = 42;
int echopinL = 41;

int trigpinR = 36;
int echopinR = 35;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(921600);  //Set the baud rate to your Bluetooth module.
  pinMode(trigpinF,OUTPUT);
  pinMode(echopinF,INPUT);
  pinMode(trigpinL,OUTPUT);
  pinMode(echopinL,INPUT);
  pinMode(trigpinR,OUTPUT);
  pinMode(echopinR,INPUT);
  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(stdby,OUTPUT);
 
}

void loop() {
  // put your main code here, to run repeatedly:
 
  long durationF, distanceF;
  digitalWrite(trigpinF,HIGH);
  delay(1);
  digitalWrite(trigpinF,LOW);
  durationF=pulseIn(echopinF,HIGH);
  distanceF=(durationF/2)/29.1;
  // Serial.print("DistanceF:");
  // Serial.print(distanceF);
  // Serial.print("CM");

  long durationL, distanceL;
  digitalWrite(trigpinL,HIGH);
  delay(1);
  digitalWrite(trigpinL,LOW);
  durationL=pulseIn(echopinL,HIGH);
  distanceL=(durationL/2)/29.1;
  // Serial.print(" DistanceL:");
  // Serial.print(distanceL);
  // Serial.print("CM ");

  long durationR, distanceR;
  digitalWrite(trigpinR,HIGH);
  delay(1);
  digitalWrite(trigpinR,LOW);
  durationR=pulseIn(echopinR,HIGH);
  distanceR=(durationR/2)/29.1;
  // Serial.print("DistanceR:");
  // Serial.print(distanceR);
  // Serial.println("CM ");


  if( (distanceR<5 && distanceL>=5 && distanceF>=5) || (distanceL>15 && distanceR<9 && distanceF<5) || (distanceL<5 && distanceR>=5 && distanceF>=5) || (distanceR>15 && distanceL<9 && distanceF<=5)  ||(distanceF>=5 && distanceL<=18 && distanceL>=5 && distanceR>=5 && distanceR<=18)|| (distanceF<=3 ||distanceF>1000) || (distanceF<=5 && distanceR<=5 && distanceL<=5) || (distanceL<=3 || distanceL>1000) || (distanceR<=3 || distanceR>1000)  )
    {
  if (distanceR<5 && distanceL>=5 && distanceF>=5)  { left();
                                    }
  else if (distanceL>15 && distanceR<9 && distanceF<5)  {  
                                      left();
                                      delay(400);
                                      forward();
                                      delay(800);
                                      Serial.print(" L ");
                                    }
  if (distanceL<5 && distanceR>=5 && distanceF>=5)  { right();
                                    }
  else if (distanceR>15 && distanceL<9 && distanceF<=5)  {  
                                      right();
                                      delay(400);
                                      forward();
                                      delay(800);
                                      Serial.print(" R ");
                                    }                                                                      
  if (distanceF>=5 && distanceL<=18 && distanceL>=5 && distanceR>=5 && distanceR<=18) {  forward();  }

  //  if ((distanceF>=20 && distanceL>=20)||(distanceR>=20 && distanceL>=20)||(distanceF>=20 && distanceR>=20))  { 
  //                       //step0();
  //                       stop();
  //                       delay(5000);
  //                                   }

  if (distanceF<=3 || distanceF>1000){
                      Serial.print(" B ");
                      backward();
                      delay(100);                  
                      if(distanceR>distanceL){right();
                                              delay(400);}
                      if(distanceL>distanceR){left();
                                              delay(400);} 
                      }                         
                            
  if (distanceL<=3 || distanceL>1000){right();
                      delay(15);} 
  if (distanceR<=3 || distanceR>1000){left();
                      delay(15);}
  if (distanceF<=5 && distanceR<=5 && distanceL<=5){
                                                    right();
                                                    delay(900);
                                                    Serial.print(" U ");
                                                  }                   
    }else{
      Serial.print(" D ");
      forward();
      delay(5);
    }                                  

                                    
  // if (distanceF>7 && distanceL>12 && distanceR>12)  { delay(450);
  //                                                     left();
  //                                                     delay(450);
  //                                                     forward();
  //                                                     delay(600);
  //                                    }
  // if (distanceF<5 && distanceL<12)  {
  //                                      right();
  //                                      delay(450);
  //                                      forward();
  //                                      delay(600);
  //                                    }

  // backward();
  // delay(2000);
  // forward();
  // delay(2000);

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
