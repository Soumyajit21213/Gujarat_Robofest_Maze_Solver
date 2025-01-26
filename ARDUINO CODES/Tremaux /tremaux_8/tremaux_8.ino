#include "helpers_2.c"

// Pin definitions and robot configuration
const int in1 = 16, in2 = 17, in3 = 8, in4 = 3, enA = 15, enB = 46, stdby = 18;
const int trigPinLeft = 42, echoPinLeft = 41, trigPinCentre = 40, echoPinCentre = 37, trigPinRight = 36, echoPinRight = 35;
const int trigPinLeftmid = 39, echoPinLeftmid = 19, trigPinRightmid = 43, echoPinRightmid = 20;
int speeda = 180, speedb = 180;
const int encoderA1 = 5, encoderA2 = 6, encoderB1 = 12, encoderB2 = 11;
long aposi = 0, bposi = 0;
float wheelCircumference = 13.8;  // circumference = 2*pie*2.2 cm
float unitLength = 28;
int block = 374 * unitLength / wheelCircumference;

int corr_back =false;
bool backflag = false;
float lentotime = 51.326138;
int width = 12;
int length = 14;
int forward_threshold = ((unitLength - length)/2);
bool wall_check[3] = { false, false, false };
ManualMap maze = {};
uint8_t dir = 0;
int X = 0, Y = 0;
Position previ;

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
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinCentre, OUTPUT);
  pinMode(echoPinCentre, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  pinMode(trigPinRightmid, OUTPUT);
  pinMode(echoPinRightmid, INPUT);
  pinMode(trigPinLeftmid, OUTPUT);
  pinMode(echoPinLeftmid, INPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(stdby, OUTPUT);
  Serial.println("Starting Trémaux Algorithm Simulation...");

  pinMode(encoderA1, INPUT);  // Set pin mode for encoderA1 (encoder A)
  pinMode(encoderA2, INPUT);  // Set pin mode for encoderA2 (encoder B)
  pinMode(encoderB1, INPUT);  // Set pin mode for encoderA1 (encoder A)
  pinMode(encoderB2, INPUT);  // Set pin mode for encoderA2 (encoder B)
  attachInterrupt(digitalPinToInterrupt(encoderA1), handleEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderB1), handleEncoderB, RISING);

  forward();delay(unitLength*lentotime-6);
}
void handleEncoderA() {
  int a = digitalRead(encoderA2);  // Read the state of encoderA2 (encoder B)
  if (a > 0) {
    aposi++;  // Increment position if encoderA2 is HIGH
  } else {
    aposi--;  // Decrement position if encoderA2 is LOW
  }
}
void handleEncoderB() {
  int b = digitalRead(encoderB2);  // Read the state of encoderA2 (encoder B)
  if (b > 0) {
    bposi++;  // Increment position if encoderA2 is HIGH
  } else {
    bposi--;  // Decrement position if encoderA2 is LOW
  }
}

void backward() {
  digitalWrite(stdby, HIGH);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  Serial.println(" backward");
  analogWrite(enA, speeda);
  analogWrite(enB, speedb);
}
void forward() {
  digitalWrite(stdby, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  Serial.println(" forward");
  analogWrite(enA, speeda);
  analogWrite(enB, speedb);
}
void right() {
  digitalWrite(stdby, HIGH);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  Serial.println(" right");
  analogWrite(enA, speeda);
  analogWrite(enB, speedb);
}
void left() {
  digitalWrite(stdby, HIGH);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  Serial.println(" left");
  analogWrite(enA, speeda);
  analogWrite(enB, speedb);
}
void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(stdby, LOW);
  Serial.println(" stop");
}

void step0() {
  if ((!wall_check[0] && !wall_check[1]) || (!wall_check[0] && !wall_check[2]) || (!wall_check[2] && !wall_check[1]) ) {
    uint8_t minVisits = UINT8_MAX;
    char bestMove = 'x';
    uint8_t chosen = 5;
    MazeWall *Cell;
    if (backflag == true) {
      Cell = get_top(&maze);
    } else {
      Cell = get(&maze, make_pair(X, Y));
    }
    
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
      MazeWall newCell = { 0, 0, 0, 0 };
      insert(&maze, currentPos, newCell);
      Cell = get(&maze, currentPos);
    }

    // Update the cell's travelled direction
    if (chosen == 0) Cell->travelled[(dir + 0) % 4]++;
    else if (chosen == 1) Cell->travelled[(dir + 1) % 4]++;
    else if (chosen == 2) Cell->travelled[(dir + 3) % 4]++;
    Cell->travelled[(dir + 2) % 4]++;  // Increment opposite direction for backtracking
    if ((Cell->travelled[(dir + 2) % 4] > 0) && (!wall_check[0] && Cell->travelled[(dir + 0) % 4] > 0)
        && (!wall_check[1] && Cell->travelled[(dir + 1) % 4] > 0) && ((!wall_check[2] && Cell->travelled[(dir + 2) % 4] > 0))) {
      delete_node(&maze, currentPos);
    }
    // Update position and direction
    moveToNextDir(bestMove);
    rotToNextDir(bestMove);

    // Set previous position for tracking
    previ.x = X;
    previ.y = Y;
    previ.direction = dir;
  }
  backflag = false;
}

void updatePosition(uint8_t dir, char upd) {
  switch (dir) {
    case 0:
      if (upd == 'f') Y++;
      else if (upd == 'b') Y--;
      else if (upd == 'r') X++;
      else if (upd == 'l') X--;
      break;
    case 1:
      if (upd == 'f') X++;
      else if (upd == 'b') X--;
      else if (upd == 'r') Y--;
      else if (upd == 'l') Y++;
      break;
    case 2:
      if (upd == 'f') Y--;
      else if (upd == 'b') Y++;
      else if (upd == 'r') X--;
      else if (upd == 'l') X++;
      break;
    case 3:
      if (upd == 'f') X--;
      else if (upd == 'b') X++;
      else if (upd == 'r') Y++;
      else if (upd == 'l') Y--;
      break;
  }
}

void moveToNextDir(char move) {
  switch (move) {
    case 'r':
      dir = (dir + 1) % 4;
      Serial.println("R");
      break;
    case 'l':
      dir = (dir + 3) % 4;
      Serial.println("L");
      break;
    case 'b':
      dir = (dir + 2) % 4;
      Serial.println("B");
      break;
    case 'f': break;
    default: Serial.println("ERROR"); break;
  }
}
void rotToNextDir(char move) {
  switch (move) {
    case 'r':
      right();
      delay(380);
      Serial.println("R");
      break;
    case 'l':
      left();
      delay(380);
      Serial.println("L");
      break;
    //case 'b': right();delay(400);right();delay(400);Serial.println("B"); break;
    case 'f': break;
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
  int distance = (duration * 0.034) / 2;  // Convert to cm
  return distance;
}


void loop() {
  int apos = aposi, bpos = bposi;
  wall_check[0] = 0;
  wall_check[1] = 0;
  wall_check[2] = 0;
  int prevturn =0;
  int distanceF, distanceL, distanceR, distanceLF, distanceRF,prevF;
  // Read Front Sensor
  distanceL = getUltrasonicDistance(trigPinLeft, echoPinLeft);
  distanceF = getUltrasonicDistance(trigPinCentre, echoPinCentre);
  distanceR = getUltrasonicDistance(trigPinRight, echoPinRight);
  distanceLF = getUltrasonicDistance(trigPinLeftmid, echoPinLeftmid);
  distanceRF = getUltrasonicDistance(trigPinRightmid, echoPinRightmid);


  if (((distanceR <= (unitLength - width) / 2 && distanceF >= (unitLength - length) && distanceL <= (unitLength - width))||(distanceL >= unitLength && distanceR <= unitLength - (width / 2) + 3 && distanceF < forward_threshold)||(distanceL <= (unitLength - width) / 2&& distanceF >= (unitLength - length) && distanceR <= (unitLength - width))||(distanceR >= unitLength && distanceL <= unitLength - (width / 2) + 3 && distanceF < forward_threshold)||((distanceR <= unitLength - width||(distanceR>200&&distanceRF<=unitLength-width)) && (distanceL <= unitLength - width || (distanceL>200&&distanceLF<=unitLength-width))&& distanceF < forward_threshold)||((distanceR >= unitLength && distanceL >= unitLength) || (distanceL >= unitLength && distanceF >= unitLength) || (distanceR >= unitLength && distanceF >= unitLength)))&&corr_back==false) {


    if (distanceR <= (unitLength - width) / 2 && distanceF >= (unitLength - length) && distanceL <= (unitLength - width)) {
      prevturn=0;
      left();
      delay(35);
      stop();
    } else if (distanceL >= unitLength && distanceR <= unitLength - (width / 2) + 3 && distanceF < forward_threshold) {
      prevturn=0;
      left();
      delay(380);
      moveToNextDir('l');
      forward();
      delay((2*unitLength/3) * lentotime);
      updatePosition(dir, 'f');
      Serial.print(" L ");
    }
    if (distanceL <= (unitLength - width) / 2&& distanceF >= (unitLength - length) && distanceR <= (unitLength - width)) {
      prevturn =1;
      right();
      delay(35);
      stop();
    }

    else if (distanceR >= unitLength && distanceL <= unitLength - (width / 2) + 3 && distanceF < forward_threshold) {
      prevturn =1;
      right();
      delay(380);
      moveToNextDir('r');
      forward();
      delay((2*unitLength/3) * lentotime);
      updatePosition(dir, 'f');
      Serial.print(" R ");
    }
    if ((distanceR <= unitLength - width||(distanceR>200&&distanceRF<=unitLength-width)) && (distanceL <= unitLength - width || (distanceL>200&&distanceLF<=unitLength-width))&& distanceF < forward_threshold)  {
      if(distanceR>distanceL){
        right();
        delay(770);
      } else {
        left();
        delay(770);
      }
      moveToNextDir('b');
      backflag = true;
    }
    if ((distanceR >= unitLength && distanceL >= unitLength) || (distanceL >= unitLength && distanceF >= unitLength) || (distanceR >= unitLength && distanceF >= unitLength)) {
      stop();
      bool last_turn = 0;
      if (distanceLF > distanceRF) {
        last_turn = 1;
        left();
        delay(150);
        stop();
      } else {
        last_turn = 0;
        right();
        delay(150);
        stop();
      }
      prevF=distanceF;
      distanceL = getUltrasonicDistance(trigPinLeft, echoPinLeft);
      distanceF = getUltrasonicDistance(trigPinCentre, echoPinCentre);
      distanceR = getUltrasonicDistance(trigPinRight, echoPinRight);
      wall_check[0] = (distanceF <= unitLength);
      wall_check[1] = (distanceR <= unitLength);
      wall_check[2] = (distanceL <= unitLength);

      if ((distanceR >= unitLength && distanceL >= unitLength) || (distanceL >= unitLength && distanceF >= unitLength) || (distanceR >= unitLength && distanceF >= unitLength)) {
        if (last_turn == 1) {
          right();
          delay(150);
          stop();
          delay(1000);
          forward();delay((lentotime * length/2)+2);
          step0();
          forward();
          delay((2*unitLength/3) * lentotime);
          updatePosition(dir, 'f');
        } else {
          left();
          delay(150);
          forward();
          stop();
          delay(500);
          forward();delay((lentotime * length/2)+2);
          step0();
          forward();
          delay((2*unitLength/3) * lentotime);
          updatePosition(dir, 'f');
        }
      }
    }
  } else {
    forward();
    delay(20);  // to detect change in encoders 
    updatePosition(dir, 'f');
    corr_back=false;     //if corr_back == true; it comes in else 
  }
  
  if(apos==aposi && bpos==bposi){
  corr_back=true;
  stop();
  backward();delay(200);
  if(prevturn==0){right();delay(200);forward();delay(40);}
  else{left();delay(200);forward();delay(40);}
 }
}