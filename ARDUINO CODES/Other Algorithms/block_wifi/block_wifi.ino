#include <WiFi.h>
#include <WebServer.h>

int in1 = 16;
int in2 = 17;
int in3 = 8;
int in4 = 3;
int enA = 15;
int enB = 46;
int speeda = 150;
int speedb = 150;
int stdby = 18;

float unitLength = 22;
int width = 12;
int length =15;

int trigpinF = 40;
int echopinF = 37;

int trigpinL = 42;
int echopinL = 41;

int trigpinR = 36;
int echopinR = 35;

const char* ssid = "ESP32_Ultrasonic";
const char* password = "12345678";

// Create a web server object
WebServer server(80);

void handleRoot();
void handleDistance();

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

  // Start serial communication
  Serial.begin(115200);

  // Start Wi-Fi access point
  WiFi.softAP(ssid, password);
  Serial.println("WiFi AP Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Define routes for the web server
  server.on("/", handleRoot);
  server.on("/distance", handleDistance);

  // Start the web server
  server.begin();
  Serial.println("Web server started");
 
}

void handleDistance(){
  // put your main code here, to run repeatedly:
 
  long durationF, distanceF;
  digitalWrite(trigpinF,HIGH);
  delay(1);
  digitalWrite(trigpinF,LOW);
  durationF=pulseIn(echopinF,HIGH);
  distanceF=(durationF/2)/29.1;

  long durationL, distanceL;
  digitalWrite(trigpinL,HIGH);
  delay(1);
  digitalWrite(trigpinL,LOW);
  durationL=pulseIn(echopinL,HIGH);
  distanceL=(durationL/2)/29.1;

  long durationR, distanceR;
  digitalWrite(trigpinR,HIGH);
  delay(1);
  digitalWrite(trigpinR,LOW);
  durationR=pulseIn(echopinR,HIGH);
  distanceR=(durationR/2)/29.1;


if((distanceR<=(unitLength-width)/2 && distanceF>=unitLength && distanceL<=unitLength-width)||(distanceL>=unitLength && distanceR<=unitLength-width&& distanceF<=unitLength-length)||(distanceL<=(unitLength-width)/2 && distanceF>=unitLength && distanceR<=unitLength-width)||(distanceR>=unitLength && distanceL<=unitLength-width&& distanceF<=unitLength-length)||(distanceR>=unitLength&&distanceL>=unitLength)||(distanceL>=unitLength&&distanceF>=unitLength)||(distanceR>=unitLength&&distanceF>=unitLength)){

  if (distanceR<=(unitLength-width)/2 && distanceF>=unitLength && distanceL<=unitLength-width)  { left();
                                    }
  else if (distanceL>=unitLength && distanceR<=unitLength-(width/2) +3&& distanceF<=unitLength-length)  {  
                                       left();
                                      delay(400);
                                      forward();
                                      delay(800);
                                      Serial.print(" L ");
                                    }
  if (distanceL<=(unitLength-width)/2 && distanceF>=unitLength && distanceR<=unitLength-width)  { right();
                                    }
  else if (distanceR>=unitLength && distanceL<=unitLength-(width/2)+3&& distanceF<=unitLength-length)  {  
                                      right();
                                      delay(400);
                                      forward();
                                      delay(800);
                                      Serial.print(" R ");
                                    }     
      if((distanceR>=unitLength&&distanceL>=unitLength)||(distanceL>=unitLength&&distanceF>=unitLength)||(distanceR>=unitLength&&distanceF>=unitLength)){stop();delay(2000);}                                                                                         
  // if (distanceF>=5 && distanceL<=18 && distanceL>=5 && distanceR>=5 && distanceR<=18) {  forward();  }
}
else{forward();}

String jsonResponse = "{";
  jsonResponse += "\"left\":" + String(distanceL) + ",";
  jsonResponse += "\"centre\":" + String(distanceF) + ",";
  jsonResponse += "\"right\":" + String(distanceR);
  jsonResponse += "}";

  server.send(200, "application/json", jsonResponse);

}

void handleRoot() {
  String html = "<!DOCTYPE html>\n<html>\n<head>\n<title>Ultrasonic Distances</title>\n";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n";
  html += "<style>\nbody { font-family: Arial, sans-serif; text-align: center; padding: 20px; }\n";
  html += "h1 { font-size: 2.5em; margin-bottom: 20px; }\n";
  html += "#distances { font-size: 2em; margin-top: 20px; }\n</style>\n</head>\n<body>\n";
  html += "<h1>Ultrasonic Distances</h1>\n";
  html += "<div id=\"distances\">\n";
  html += "  <p>Distance Left: <span id=\"distanceLeft\">Fetching...</span> cm</p>\n";
  html += "  <p>Distance Centre: <span id=\"distanceCentre\">Fetching...</span> cm</p>\n";
  html += "  <p>Distance Right: <span id=\"distanceRight\">Fetching...</span> cm</p>\n";
  html += "</div>\n";
  html += "<script>\n";
  html += "function updateDistances() {\n";
  html += "  fetch('/distance')\n";
  html += "    .then(response => response.json())\n";
  html += "    .then(data => {\n";
  html += "      document.getElementById('distanceLeft').innerText = data.left;\n";
  html += "      document.getElementById('distanceCentre').innerText = data.centre;\n";
  html += "      document.getElementById('distanceRight').innerText = data.right;\n";
  html += "    });\n";
  html += "}\n";
  html += "setInterval(updateDistances, 100);\n";
  html += "</script>\n";
  html += "</body>\n</html>";

  server.send(200, "text/html", html);
}

void loop() {
  // Handle incoming web requests
  server.handleClient();
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
