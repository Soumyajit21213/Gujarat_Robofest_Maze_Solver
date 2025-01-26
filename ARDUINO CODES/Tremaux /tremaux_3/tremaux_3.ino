#include "helpers_2.c"  


// Pin definitions and robot configuration
const int in1 = 16, in2 = 17, in3 = 8, in4 = 3, enA = 15, enB = 46, stdby = 18;
const int trigPinLeft = 42, echoPinLeft = 41, trigPinCentre = 40, echoPinCentre = 37, trigPinRight = 36, echoPinRight = 35;
int speeda = 150, speedb = 150;
bool backflag=false;
float lentotime= 1400/29.2;
float unitLength = 23;
int width = 12;
int length =14;
int forward_threshold = unitLength-length-(unitLength-length)/2;
bool wall_check[3]={false,false,false};
uint8_t wall[MAX_SIZE][MAX_SIZE] = {0}; 
uint8_t X_DIM, Y_DIM;
int target_x = -1, target_y = -1; 
ManualMap maze = {};
uint8_t dir = 0; 
int X = 0, Y = 0; 
int min_X = 0, max_X = 0, min_Y = 0, max_Y = 0;
bool backtracking = false;
Position previ;

void UpdateWalls(bool wall_check[]);
void MoveBot(char direction);
void updatePosition(uint8_t dir, char upd);
void HandleBacktracking();
void handlestack();
void handleDistance();
void handleRoot();
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
    pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
    pinMode(enA, OUTPUT); pinMode(enB, OUTPUT); pinMode(stdby, OUTPUT);
    Serial.println("Starting Trémaux Algorithm Simulation...");
   
    // Start serial communication
}

void backward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(stdby, HIGH);
  Serial.println(" backward");
  analogWrite(enA,speeda);
  analogWrite(enB,speedb);

}
void forward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(stdby, HIGH);
  Serial.println(" forward");
  analogWrite(enA,speeda);
  analogWrite(enB,speedb);

}
void right() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(stdby, HIGH);
  Serial.println(" right");
  analogWrite(enA,speeda);
  analogWrite(enB,speedb);

}
void left() { 
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  digitalWrite(stdby, HIGH);
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

void step0(){
    if((!wall_check[0]&&!wall_check[1])||(!wall_check[0]&&!wall_check[2])||(!wall_check[2]&&!wall_check[1])){
       //stop();delay(2000);Serial.println("Junction wala stop");
       UpdateWalls(wall_check);
       uint8_t minVisits = UINT8_MAX;
       char bestMove = 'x';
  
        if (!wall_check[0]) {
                uint8_t visits = IsNumbered(dir, 'f');
                if (visits < minVisits) {
                    minVisits = visits;
                    bestMove = 'f';
                }
            }
        if (!wall_check[1]) {
                uint8_t visits = IsNumbered(dir, 'r');
                if (visits < minVisits) {
                    minVisits = visits;
                    bestMove = 'r';
                }
            }
        if (!wall_check[2]) {
                uint8_t visits = IsNumbered(dir, 'l');
                if (visits < minVisits) {
                    minVisits = visits;
                    bestMove = 'l';
                }
            }

        moveToNextDir(bestMove);
        rotToNextDir(bestMove);
        Pair currentPos = make_pair(X, Y);
        MazeWall *currentCell = get(&maze, currentPos);
        if (!currentCell) {
            MazeWall newCell = {0, 1};
            insert(&maze, currentPos, newCell);
        } else {
            currentCell->visited++;
        }
        previ = {X, Y, dir};
        }
     
}

void updatePosition(uint8_t dir, char upd) {
    switch (dir) {
        case 0: if (upd == 'f') Y++; else if (upd == 'b') Y--; else if (upd == 'r') X++; else if (upd == 'l') X--; break;
        case 1: if (upd == 'f') X++; else if (upd == 'b') X--; else if (upd == 'r') Y--; else if (upd == 'l') Y++; break;
        case 2: if (upd == 'f') Y--; else if (upd == 'b') Y++; else if (upd == 'r') X--; else if (upd == 'l') X++; break;
        case 3: if (upd == 'f') X--; else if (upd == 'b') X++; else if (upd == 'r') Y++; else if (upd == 'l') Y--; break;
    }
}

void UpdateWalls(bool wall_check[]) {
    MazeWall *current = get(&maze, make_pair(X, Y));
    if (!current) {
        MazeWall newCell = {0, 0};
        insert(&maze, make_pair(X, Y), newCell);
        current = get(&maze, make_pair(X, Y));
    }

    if (current->visited == 0) {
        int Wall_state = 0;
        switch (dir) {
            case 0: if (wall_check[0]) Wall_state |= 1; if (wall_check[1]) Wall_state |= 2; if (wall_check[2]) Wall_state |= 8; break;
            case 1: if (wall_check[0]) Wall_state |= 2; if (wall_check[1]) Wall_state |= 4; if (wall_check[2]) Wall_state |= 1; break;
            case 2: if (wall_check[0]) Wall_state |= 4; if (wall_check[1]) Wall_state |= 8; if (wall_check[2]) Wall_state |= 2; break;
            case 3: if (wall_check[0]) Wall_state |= 8; if (wall_check[1]) Wall_state |= 1; if (wall_check[2]) Wall_state |= 4; break;
        }
        current->walls = Wall_state;
    }
    current->visited++;
}

bool IsNumbered(uint8_t dir, char direction) {
    uint8_t newX = X, newY = Y;
    switch (dir) {
        case 0: if (direction == 'f') newY++; else if (direction == 'r') newX++; else if (direction == 'l') newX--; break;
        case 1: if (direction == 'f') newX++; else if (direction == 'r') newY--; else if (direction == 'l') newY++; break;
        case 2: if (direction == 'f') newY--; else if (direction == 'r') newX--; else if (direction == 'l') newX++; break;
        case 3: if (direction == 'f') newX--; else if (direction == 'r') newY++; else if (direction == 'l') newY--; break;
    }
    MazeWall *nextCell = get(&maze, make_pair(newX, newY));
    return (nextCell && nextCell->visited > 0);
}

char getMove(uint8_t currentDir, uint8_t lastDir) {
    uint8_t targetDir = (lastDir + 2) % 4;
    uint8_t diff = (targetDir - currentDir + 4) % 4;
    switch (diff) {
        case 0: return 'f';
        case 1: return 'r';
        case 3: return 'l';
        case 2: return 'b';
        default: return 'N';
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
        case 'r':right();delay(400);Serial.println("R"); break;
        case 'l':left(); delay(400);Serial.println("L");  break;
       // case 'b': right();delay(400);right();delay(400);Serial.println("B"); break;
        case 'f':break;
        default: Serial.println("ERROR"); break;
    }
}

/*void displayMap(const ManualMap* map) {
    // Serial.println("ManualMap contents:");
    for (uint8_t i = 0; i < maze.size; ++i) {
        Serial.print(maze.data[i].coord.x);
        Serial.print(" ");
        Serial.print(maze.data[i].coord.y);
        Serial.print(" ");
        Serial.print(maze.data[i].value.walls);
        Serial.print(" ");
        Serial.println(maze.data[i].value.visited);
    }
    Serial.println();
}*/

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
    wall_check[0] = 0; 
    wall_check[1] = 0; 
    wall_check[2] = 0;
    
    int distanceF, distanceL, distanceR;
    // Read Front Sensor
    distanceL = getUltrasonicDistance(trigPinLeft, echoPinLeft);
    distanceF = getUltrasonicDistance(trigPinCentre, echoPinCentre);
    distanceR = getUltrasonicDistance(trigPinRight, echoPinRight);

    // Display the wall_check results
  /*  Serial.print("Wall Check: Front=");
    Serial.print(wall_check[0]);
    Serial.print(", Left=");
    Serial.print(wall_check[2]);
    Serial.print(", Right=");
    Serial.println(wall_check[1]);*/

    // Handle conditions
    /*if (distanceF > 500 && distanceR > 500 && distanceL > 500) {
        target_x = previ.x;
        target_y = previ.y;
        Serial.println("DONE");
    }*/
    //Serial.print(X);Serial.print(" ");Serial.print(Y);Serial.print(" ");Serial.print(dir);Serial.print("\n");


if((distanceR<=(unitLength-width)/2 && distanceF>=forward_threshold && distanceL<=unitLength-width)||(distanceL>=unitLength && distanceR<=unitLength-width&& distanceF<=unitLength-length)||(distanceL<=(unitLength-width)/2 && distanceF>=forward_threshold&& distanceR<=unitLength-width)||(distanceR>=unitLength && distanceL<=unitLength-width&& distanceF<=unitLength-length)||(distanceR>=unitLength&&distanceL>=unitLength)||(distanceL>=unitLength&&distanceF>=unitLength)||(distanceR>=unitLength&&distanceF>=unitLength)||distanceL>=300||distanceR>=300||(distanceR<=unitLength-width&&distanceL<=unitLength-width&&distanceF<=unitLength-length)){


  if ((distanceR<=(unitLength-width)/2&& distanceF>=forward_threshold && distanceL<=unitLength-width)||distanceR>=300)  { left();delay(20);stop();
                                    }
  else if (distanceL>=unitLength && distanceR<=unitLength-(width/2) +3&& distanceF<=forward_threshold)  {  
                                       left();
                                       moveToNextDir('l');
                                        delay(400);
                                      forward();
                                      updatePosition(dir,'f');
                                      delay(lentotime*unitLength);
                                      Serial.print(" L ");
                                    }
  if ((distanceL<=(unitLength-width)/2 && distanceF>=forward_threshold && distanceR<=unitLength-width)||distanceL>=300)  { right();delay(20);stop();
                                    }
  else if (distanceR>=unitLength && distanceL<=unitLength-(width/2)+3&& distanceF<=forward_threshold)  {  
                                      right();
                                      delay(400);
                                      moveToNextDir('r');
                                      forward();
                                      updatePosition(dir,'f');
                                      delay(unitLength*lentotime);
                                      Serial.print(" R ");
                                    }      
   if(distanceR<=unitLength-width&&distanceL<=unitLength-width&&distanceF<=forward_threshold){right();delay(400);right();delay(400);}                                                         
  if((distanceR>=unitLength&&distanceL>=unitLength)||(distanceL>=unitLength&&distanceF>=unitLength)||(distanceR>=unitLength&&distanceF>=unitLength)){
           bool last_turn =0 ;
           int tempL=distanceL,tempR=distanceR,tempF=distanceF;
           if(distanceL>distanceR){last_turn =1;left();delay(100);stop();}
           else{last_turn =0;right();delay(100);stop();}
           distanceL = getUltrasonicDistance(trigPinLeft, echoPinLeft);
           distanceF = getUltrasonicDistance(trigPinCentre, echoPinCentre);
           distanceR = getUltrasonicDistance(trigPinRight, echoPinRight);
           wall_check[0]=(distanceF<20);
           wall_check[1]=(distanceR<15);
           wall_check[2]=(distanceL<15);

  if((distanceR>=unitLength&&distanceL>=unitLength)||(distanceL>=unitLength&&distanceF>=unitLength)||(distanceR>=unitLength&&distanceF>=unitLength))
              {if(last_turn==1){right();delay(100);stop();delay(2000);step0();forward();delay(lentotime*unitLength);}
                else{left();delay(100);stop();delay(2000);step0();forward();delay(unitLength*lentotime);}
              }}}
else{forward();delay(unitLength*lentotime);updatePosition(dir,'f');}                     
    //displayMap(&maze);
    //Serial.print(X);Serial.print(" ");Serial.print(Y);Serial.print(" ");Serial.print(dir);Serial.print("\n");
}