#include "helpers_2.c"   

// Pin definitions and robot configuration
const int in1 = 16, in2 = 17, in3 = 8, in4 = 3, enA = 15, enB = 46, stdby = 18;
const int trigPinLeft = 42, echoPinLeft = 41, trigPinCentre = 40, echoPinCentre = 37, trigPinRight = 36, echoPinRight = 35;
int speeda = 150, speedb = 150;
bool backflag=false;

// Global variables
Stack stack;
float lentotime= 1400/29.2;
float unitLength = 23;
int width = 12;
int length =14;
int forward_threshold = unitLength-length-(unitLength-length)/2;
bool wall_check[] = {false, false, false};
int dir = 0, X = 0, Y = 0;
Pair prevcoor = {0, 0};


// Function prototypes
void moveToNextDir(char move);
void updatePosition(uint8_t dir, char move);
void insertNode(Pair coord, uint8_t dir, char open[], uint8_t open_count[]);
char getMove(uint8_t currentDir, uint8_t targetDir);
int getUltrasonicDistance(int trigPin, int echoPin);
void initStack();
//void step0();
void setup();
void loop();

// Print stack for debugging
/*void print_stack(Stack* stack) {
    for (int i = 0; i < stack->top; ++i) {
        Serial.print("Node "); Serial.print(i);
        Serial.print(": ("), Serial.print(stack->nodes[i]->coord.x);
        Serial.print(","), Serial.print(stack->nodes[i]->coord.y);
        Serial.print(") | Visited: "), Serial.print(stack->nodes[i]->visited_count);
        Serial.print(" | Facing: "), Serial.print(stack->nodes[i]->dir);

        Serial.print(" | Open: ");
        for (int j = 0; j < 4; j++) {
            Serial.print(stack->nodes[i]->open[j]);
            Serial.print("(");
            Serial.print(stack->nodes[i]->open_count[j]);
            Serial.print(")");
            if (j < 3) Serial.print(", ");
        }
        Serial.println();
    }
}*/

void initStack() {
    init_stack(&stack);
}
void insertNode(Pair coord, uint8_t dir, char open[], uint8_t open_count[]) {
    insert(&stack, coord, dir, open, open_count);
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

// Get move direction based on current and target direction
char getMove(uint8_t currentDir, uint8_t targetDir) {
    uint8_t diff = (targetDir - currentDir + 4) % 4;
    return (diff == 0) ? 'f' : (diff == 1) ? 'r' : (diff == 3) ? 'l' : 'b';
}

char normalize_move(char move, uint8_t dir) {
    switch (dir) {
        case 0: return move;  
        case 1: return (move == 'b') ? 'r' : (move == 'f') ? 'l' : (move == 'r') ? 'f' : 'b';
        case 2: return (move == 'b') ? 'f' : (move == 'f') ? 'b' : (move == 'r') ? 'l' : 'r';
        case 3: return (move == 'b') ? 'l' : (move == 'f') ? 'r' : (move == 'r') ? 'b' : 'f';
    }
    return '0'; 
}
// Move robot to next direction
void moveToNextDir(char move) {
    switch (move) {
        case 'r': dir = (dir + 1) % 4;right();delay(400); Serial.println("R"); break;
        case 'l': dir = (dir + 3) % 4; left();delay(400);Serial.println("L"); break;
        case 'b': dir = (dir + 2) % 4;right();delay(400);right();delay(400); Serial.println("B"); break;
        case 'f':Serial.println("F"); break;
        default: Serial.println("ERROR: Invalid move!"); break;
    }
}

// Update position based on direction and move
void updatePosition(uint8_t dir, char move) {
    int delta = (move == 'f') ? 1 : (move == 'b') ? -1 : 0;
    switch (dir) {
        case 0: Y += delta; break; // North
        case 1: X += delta; break; // East
        case 2: Y -= delta; break; // South
        case 3: X -= delta; break; // West
    }
}

void tremaux() {
    Pair curr_coor = {X, Y};
    bool visitedBefore = false;
    char open[4] = {'0', '0', '0', '0'};
    uint8_t open_count[4] = {0};
   
    //prev junction check 
   // Serial.println("entered the loop");Serial.println("X Y: ");Serial.print(X);Serial.print(Y);Serial.print(" ");
    for (int i = 0; i < stack.top; ++i){
    if (stack.nodes[i]->coord.x == X && stack.nodes[i]->coord.y == Y) {
        Serial.println("Already visited this coordinate, skipping step0.");
        visitedBefore = true;
        memcpy(open, stack.nodes[i]->open, sizeof(stack.nodes[i]->open));
        memcpy(open_count, stack.nodes[i]->open_count, sizeof(stack.nodes[i]->open_count));
        break;
     }
   }

   Serial.println("Exited the loop");
    // If all walls are present, backtrack
   /* if (wall_check[0] && wall_check[1] && wall_check[2]) {
        moveToNextDir('b');
        return;
    }*/

    if (!visitedBefore&&!backflag) {
        switch (dir) {
            case 0: // North
                open[0] = !wall_check[0] ? 'f' : '0';
                open[1] = !wall_check[1] ? 'r' : '0';
                open[3] = !wall_check[2] ? 'l' : '0';
                open[2]='b';
                break;
            case 1: // East
                open[0] = !wall_check[2] ? 'f' : '0';
                open[1] = !wall_check[0] ? 'r' : '0';
                open[2] = !wall_check[1] ? 'b' : '0';
                open[3]='l';
                break;
            case 2: // South
                open[1] = !wall_check[2] ? 'r' : '0';
                open[2] = !wall_check[0] ? 'b' : '0';
                open[3] = !wall_check[1] ? 'l' : '0';
                open[0]='f';
                break;
            case 3: // West
                open[0] = !wall_check[1] ? 'f' : '0';
                open[1] = 'r';
                open[2] = !wall_check[2] ? 'b' : '0';
                open[3] = !wall_check[0] ? 'l' : '0';
                break;
           }
       }

    
        if (dir == 0) { open_count[2]++; } // Came from South
        else if (dir == 1) { open_count[3]++; } // Came from West
        else if (dir == 2) { open_count[0]++; } // Came from North
        else if (dir == 3) { open_count[1]++; } // Came from East
    
    char move = '0';
    uint8_t relativeOrder[4] = {0, 1, 3, 2}; // Prioritize f, r, l, b relative to dir
    for (uint8_t i = 0; i < 4; ++i) {
        uint8_t relativeDir = (relativeOrder[i] + dir) % 4;
        if (open[relativeDir] != '0' && open_count[relativeDir] == 0) {
            move = open[relativeDir];
            break;
        }
    }
   // Serial.println("zero moves: ");Serial.print(move);Serial.print(" ");
    // If no unexplored moves, choose the least visited one in relative direction order
    if (move == '0') {
        uint8_t min_visits = 100;
        for (uint8_t i = 0; i < 4; ++i) {
            uint8_t relativeDir = (relativeOrder[i] + dir) % 4;
            if (open[relativeDir] != '0' && open_count[relativeDir] < min_visits) {
                min_visits = open_count[relativeDir];
                move = open[relativeDir];
            }
        }
    }

    if (move != '0') {
        char normMove = normalize_move(move, dir);
        for (uint8_t i = 0; i < 4; ++i) {
            if (open[i] ==move) { open_count[i]++;break;}
        }
      //Serial.println("normmove: ");Serial.print(normMove);Serial.print(" ");
        if (visitedBefore){update_node(&stack, curr_coor, dir, open, open_count);} 
        else {insertNode(curr_coor, dir, open, open_count);}
        prevcoor = curr_coor;
        if (normMove != 'f'){ moveToNextDir(normMove);}
    }
    backflag=false;
}

void setup() {
    Serial.begin(9600);
    pinMode(trigPinLeft, OUTPUT); pinMode(echoPinLeft, INPUT);
    pinMode(trigPinCentre, OUTPUT); pinMode(echoPinCentre, INPUT);
    pinMode(trigPinRight, OUTPUT); pinMode(echoPinRight, INPUT);
    pinMode(in1, OUTPUT); pinMode(in2, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
    pinMode(enA, OUTPUT); pinMode(enB, OUTPUT); pinMode(stdby, OUTPUT);
    Serial.println("Starting Trémaux Algorithm Simulation...");
    prevcoor = {0, 0};
    initStack();
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
              {if(last_turn==1){right();delay(100);stop();delay(2000);forward();delay(lentotime*unitLength);}
                else{left();delay(100);stop();delay(2000);forward();delay(unitLength*lentotime);}
              }}}
else{forward();updatePosition(dir,'f');}                     
    //displayMap(&maze);
    //Serial.print(X);Serial.print(" ");Serial.print(Y);Serial.print(" ");Serial.print(dir);Serial.print("\n");
}