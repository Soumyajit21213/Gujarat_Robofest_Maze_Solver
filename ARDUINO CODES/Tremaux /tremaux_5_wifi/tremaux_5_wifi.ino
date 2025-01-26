#include "helpers_2.c"  

// Pin definitions and robot configuration
const int in1 = 16, in2 = 17, in3 = 8, in4 = 3, enA = 15, enB = 46, stdby = 18;
const int trigPinLeft = 42, echoPinLeft = 41, trigPinCentre = 40, echoPinCentre = 37, trigPinRight = 36, echoPinRight = 35;
const int   trigPinLeftmid=39,echoPinLeftmid=19, trigPinRightmid=43,echoPinRightmid=20;
int speeda = 150, speedb = 150;
const int encoderA1 = 5,encoderA2 = 6,encoderB1 = 12,encoderB2 = 11;
long aposi = 0,bposi = 0; 
float wheelCircumference = 13.8; // circumference = 2*pie*2.2 cm
float unitLength = 23;
int block = 374 * unitLength/wheelCircumference;
long previousTimeA = 0;
float ePreviousA = 0;
long previousTimeB = 0;
float ePreviousB = 0;
float kpa = 2.0;
float kda = 0.1;
float kpb = 2.0;
float kdb = 0.1;

bool backflag=false;
float lentotime= 1400/29.2;
int width = 12;
int length =14;
int forward_threshold = unitLength-length-(unitLength-length)/2;
bool wall_check[3]={false,false,false};
ManualMap maze = {};
uint8_t dir = 0; 
int X = 0, Y = 0; 
Position previ;

//void UpdateWalls(bool wall_check[]);
void updatePosition(uint8_t dir, char upd);
bool IsNumbered(uint8_t dir, char direction);
char getMove(uint8_t currentDir, uint8_t lastDir);
void moveToNextDir(uint8_t current_X, uint8_t current_Y, char move);
void step0();
void stop();
void forward();
void backward();
void left();
void right();

void setup() {
    Serial.begin(115200);
    pinMode(trigPinLeft, OUTPUT); pinMode(echoPinLeft, INPUT);
    pinMode(trigPinCentre, OUTPUT); pinMode(echoPinCentre, INPUT);
    pinMode(trigPinRight, OUTPUT); pinMode(echoPinRight, INPUT);
    pinMode(trigPinRightmid, OUTPUT); pinMode(echoPinRightmid, INPUT);
    pinMode(trigPinLeftmid, OUTPUT); pinMode(echoPinLeftmid, INPUT);
    pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
    pinMode(enA, OUTPUT); pinMode(enB, OUTPUT); pinMode(stdby, OUTPUT);
    Serial.println("Starting Trémaux Algorithm Simulation...");
 
  pinMode(encoderA1, INPUT); // Set pin mode for encoderA1 (encoder A)
  pinMode(encoderA2, INPUT); // Set pin mode for encoderA2 (encoder B)
  pinMode(encoderB1, INPUT); // Set pin mode for encoderA1 (encoder A)
  pinMode(encoderB2, INPUT); // Set pin mode for encoderA2 (encoder B)
  attachInterrupt(digitalPinToInterrupt(encoderA1), handleEncoderA, RISING); 
  attachInterrupt(digitalPinToInterrupt(encoderB1), handleEncoderB, RISING);
   
    // Start serial communication
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

void backward() {
  digitalWrite(stdby, HIGH);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  Serial.println(" backward");
  analogWrite(enA,speeda);
  analogWrite(enB,speedb);

}
void forward() {
  digitalWrite(stdby, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  Serial.println(" forward");
  analogWrite(enA,speeda);
  analogWrite(enB,speedb);

}
void right() {
  digitalWrite(stdby, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  Serial.println(" right");
  analogWrite(enA,speeda);
  analogWrite(enB,speedb);

}
void left() { 
  digitalWrite(stdby, HIGH);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  Serial.println(" left");
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

// void Left(int angle) {
//   digitalWrite(stdby, HIGH); // Enable the motor driver
//   int tolerance = 35;

//   float turningDistance = (PI * 9.4 * angle) / 360.0; 
//   int rotationDistance = 374 * turningDistance / wheelCircumference ;
//   int atarget = aposi - rotationDistance; // Left wheel backward
//   int btarget = bposi + rotationDistance; // Right wheel forward
//   int aError = atarget - aposi;
//   int bError = btarget - bposi;
//   float aSpeed;
//   float bSpeed;

//   while (abs(aError) > tolerance || abs(bError) > tolerance) {
//     aSpeed = pidControllerA(aError);
//     bSpeed = pidControllerB(bError);
//     digitalWrite(in2, LOW);
//     digitalWrite(in1, HIGH);
//     analogWrite(enA, constrain((int)fabs(aSpeed), 0, 200));
//     digitalWrite(in4, HIGH);
//     digitalWrite(in3, LOW);
//     analogWrite(enB, constrain((int)fabs(bSpeed), 0, 200));
//     aError = atarget - aposi;
//     bError = btarget - bposi;
//   }
//   stop();
// }

// void Right(int angle) {
//   digitalWrite(stdby, HIGH); // Enable the motor driver
//   int tolerance = 35; 
//   float turningDistance = (PI * 9.4 * angle) / 360.0; 
//   int rotationDistance = 374 * turningDistance / wheelCircumference ;

//   int atarget = aposi + rotationDistance; // Left wheel forward
//   int btarget = bposi - rotationDistance; // Right wheel backward
//   int aError = atarget - aposi;
//   int bError = btarget - bposi;
//   float aSpeed;
//   float bSpeed;

//   while (abs(aError) > tolerance || abs(bError) > tolerance) {
//     aSpeed = pidControllerA(aError);
//     bSpeed = pidControllerB(bError);
//     digitalWrite(in2, HIGH);
//     digitalWrite(in1, LOW);
//     analogWrite(enA, constrain((int)fabs(aSpeed), 0, 200));
//     digitalWrite(in4, LOW);
//     digitalWrite(in3, HIGH);
//     analogWrite(enB, constrain((int)fabs(bSpeed), 0, 200));                     
//     aError = atarget - aposi;
//     bError = btarget - bposi;
//   }
//   stop();
// }

void step0() {
    if ((!wall_check[0] && !wall_check[1]) || (!wall_check[0] && !wall_check[2]) || (!wall_check[2] && !wall_check[1])) {
        uint8_t minVisits = UINT8_MAX;
        char bestMove = 'x';
        uint8_t chosen = 5;
        MazeWall *Cell;
        if(backflag){Cell =get_top(&maze);}
        else{Cell = get(&maze, make_pair(X, Y));}

       
        if (!wall_check[0]) {
            if (Cell) {
                uint8_t visits = Cell->travelled[(dir + 0) % 4];
                if (visits < minVisits) { minVisits = visits; bestMove = 'f'; chosen = 0; }
            } else { bestMove = 'f'; chosen = 0;}
        }
        if (!wall_check[1]) {
            if (Cell) {
                uint8_t visits = Cell->travelled[(dir + 1) % 4];
                if (visits < minVisits) { minVisits = visits; bestMove = 'r'; chosen = 1; }
            } else {bestMove = 'r'; chosen = 1;}
        }
        if (!wall_check[2]) {
            if (Cell) {
                uint8_t visits = Cell->travelled[(dir + 3) % 4];
                if (visits < minVisits) { minVisits = visits; bestMove = 'l'; chosen = 2; }
            } else { bestMove = 'l'; chosen = 2;}
        }

    
        Pair currentPos = make_pair(X, Y);
        if (!Cell) {
            MazeWall newCell = {0,0,0,0};
            insert(&maze, currentPos, newCell);
            Cell = get(&maze, currentPos);
        }

        // Update the cell's travelled direction
        if (chosen == 0) Cell->travelled[(dir + 0) % 4]++;
        else if (chosen == 1) Cell->travelled[(dir + 1) % 4]++;
        else if (chosen == 2) Cell->travelled[(dir + 3) % 4]++;
        Cell->travelled[(dir + 2) % 4]++;  // Increment opposite direction for backtracking
        if( (Cell->travelled[(dir + 0) % 4]>0) && (!wall_check[0]&&Cell->travelled[(dir + 0) % 4]>0)
             && (!wall_check[1]&&Cell->travelled[(dir + 1) % 4]>0) && ((!wall_check[2]&&Cell->travelled[(dir + 2) % 4]>0))){
                    delete_node(&maze,currentPos);
        }
        // Update position and direction
        moveToNextDir(bestMove);
        rotToNextDir(bestMove);

        // Set previous position for tracking
        previ.x = X;
        previ.y = Y;
        previ.direction = dir;
    }
    backflag=false; 
}


void updatePosition(uint8_t dir, char upd) { //correct this one using encoders 
    switch (dir) {
        case 0: if (upd == 'f') Y++; else if (upd == 'b') Y--; else if (upd == 'r') X++; else if (upd == 'l') X--; break;
        case 1: if (upd == 'f') X++; else if (upd == 'b') X--; else if (upd == 'r') Y--; else if (upd == 'l') Y++; break;
        case 2: if (upd == 'f') Y--; else if (upd == 'b') Y++; else if (upd == 'r') X--; else if (upd == 'l') X++; break;
        case 3: if (upd == 'f') X--; else if (upd == 'b') X++; else if (upd == 'r') Y++; else if (upd == 'l') Y--; break;
    }
}

void moveToNextDir(char move) {
    switch (move) {
        case 'r': dir = (dir + 1) % 4;Serial.println("R"); break;
        case 'l': dir = (dir + 3) % 4;Serial.println("L");  break;
        case 'b': dir = (dir + 2) % 4;Serial.println("B"); break;
        case 'f':break;
        default: Serial.println("ERROR"); break;
    }
}
void rotToNextDir(char move) {
    switch (move) {
        case 'r':right();delay(520);Serial.println("R"); break;
        case 'l':left();delay(520);Serial.println("L");  break;
        //case 'b': right();delay(400);right();delay(400);Serial.println("B"); break;
        case 'f':break;
        default: Serial.println("ERROR"); break;
    }
}

int getUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = (duration * 0.034) / 2; // Convert to cm
  return distance;
}


void loop() {
    int apos=aposi ,bpos = bposi ;
    wall_check[0] = 0; 
    wall_check[1] = 0; 
    wall_check[2] = 0;
    bool lastturn=0;
    int distanceF, distanceL, distanceR,distanceLF,distanceRF;
    // Read Front Sensor
    distanceL = getUltrasonicDistance(trigPinLeft, echoPinLeft);
    distanceF = getUltrasonicDistance(trigPinCentre, echoPinCentre);
    distanceR = getUltrasonicDistance(trigPinRight, echoPinRight);
    distanceLF=getUltrasonicDistance(trigPinLeftmid, echoPinLeftmid);
    distanceRF=getUltrasonicDistance(trigPinRightmid, echoPinRightmid);


if(((distanceR<=(unitLength-width)/2&& distanceF>=forward_threshold && distanceL<=unitLength-width)||distanceRF<=(unitLength-width)/3)||(distanceL>=unitLength && distanceR<=unitLength-(width/2) +3&& distanceF<forward_threshold)||((distanceL<=(unitLength-width)/2 && distanceF>=forward_threshold && distanceR<=unitLength-width)||distanceLF<=(unitLength-width)/3)||(distanceR>=unitLength && distanceL<=unitLength-(width/2)+3&& distanceF<forward_threshold)||(distanceR<=unitLength-width&&distanceL<=unitLength-width&&distanceF<forward_threshold)||(distanceR>=unitLength&&distanceL>=unitLength)||(distanceL>=unitLength&&distanceF>=unitLength)||(distanceR>=unitLength&&distanceF>=unitLength)){


  if ((distanceR<=(unitLength-width)/2&& distanceF>=forward_threshold && distanceL<=unitLength-width)||distanceRF<=(unitLength-width)/3)  { left();delay(20);stop();
                                    }
  else if (distanceL>=unitLength && distanceR<=unitLength-(width/2) +3&& distanceF<forward_threshold)  {  
                                       left();delay(430);
                                       moveToNextDir('l');
                                       forward();
                                       delay(lentotime*unitLength);
                                       updatePosition(dir,'f');
                                       Serial.print(" L ");
                                       lastturn=0;
                                    }
  if ((distanceL<=(unitLength-width)/2 && distanceF>=forward_threshold && distanceR<=unitLength-width)||distanceLF<=(unitLength-width)/3){ right();delay(20);stop();
                                    }
  else if (distanceR>=unitLength && distanceL<=unitLength-(width/2)+3&& distanceF<forward_threshold)  {  
                                      right();delay(430);
                                      moveToNextDir('r');
                                      forward();
                                      delay(unitLength*lentotime);
                                      updatePosition(dir,'f');
                                      Serial.print(" R ");
                                      lastturn=1;
                                    }      
   if(distanceR<=unitLength-width&&distanceL<=unitLength-width&&distanceF<forward_threshold){
    if(distanceR>=distanceL){right();delay(800);}
    else{left();delay(800);}
    moveToNextDir('b');backflag=true;
    }                                                         
  if((distanceR>=unitLength&&distanceL>=unitLength)||(distanceL>=unitLength&&distanceF>=unitLength)||(distanceR>=unitLength&&distanceF>=unitLength)){
          /* if(distanceRF>=(unitLength-width)/2 +2&&distanceRF>=(unitLength-width)/2 +2){stop();delay(2000);}
           else{
            if(distanceLF>distanceRF){left();delay(10);stop();}
            else{right();delay(10);stop();}
            }*/
           bool last_turn =0;
           //int tempL=distanceL,tempR=distanceR,tempF=distanceF;
           if(distanceLF>distanceRF){last_turn =1;left();delay(100);stop();}
           else{last_turn =0;right();delay(100);stop();}
           distanceL = getUltrasonicDistance(trigPinLeft, echoPinLeft);
           distanceF = getUltrasonicDistance(trigPinCentre, echoPinCentre);
           distanceR = getUltrasonicDistance(trigPinRight, echoPinRight);
           wall_check[0]=(distanceF<=unitLength);
           wall_check[1]=(distanceR<=unitLength);
           wall_check[2]=(distanceL<=unitLength);

  if((distanceR>=unitLength&&distanceL>=unitLength)||(distanceL>=unitLength&&distanceF>=unitLength)||(distanceR>=unitLength&&distanceF>=unitLength))
              {if(last_turn==1){right();delay(100);stop();delay(500);step0();forward();delay(lentotime*unitLength);}
                else{left();delay(100);forward();stop();delay(500);step0();forward();delay(unitLength*lentotime);}
              }}}
else{forward();delay(20);updatePosition(dir,'f');}   
if(apos==aposi && bpos==bposi){

  backward();delay(300);
  if(distanceRF>distanceLF){right();delay(50);forward();delay(20);}
  else{left();delay(50);forward();delay(20);}
  }
}