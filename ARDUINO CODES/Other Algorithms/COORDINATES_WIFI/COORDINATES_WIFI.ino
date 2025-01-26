#include <WiFi.h>
#include <WebServer.h>

const int encoderA1 = 5;
const int encoderA2 = 6;
const int encoderB1 = 12;
const int encoderB2 = 11;
const int in1 = 16, in2 = 17, in3 = 8, in4 = 3, enA = 15, enB = 46, stdby = 18;
const int trigPinLeft = 42, echoPinLeft = 41, trigPinCentre = 40, echoPinCentre = 37, trigPinRight = 36, echoPinRight = 35;
int speeda = 150, speedb = 150;

// Wi-Fi credentials
const char* ssid = "ESP32_Ultrasonic";
const char* password = "12345678";

// Create a web server object
WebServer server(80);

const int AI2 = 16;
const int AI1 = 17;
const int Apwm = 15;
const int BI2 = 8;
const int BI1 = 3;
const int Bpwm = 46;


long aposi = 0;
long bposi = 0;
float unitLength = 22;
int width = 12;
int length =14; // 1 feet = 30.48 cm 
float wheelCircumference = 14.5; // circumference = 2*pie*2.2 cm
int block = 374 * unitLength/wheelCircumference;


void setup() {
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
void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(stdby, LOW);
  Serial.println(" stop");
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>Ultrasonic and Encoder Data</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body {
      font-family: Arial, sans-serif;
      text-align: center;
      padding: 20px;
    }
    h1 {
      font-size: 2.5em;
      margin-bottom: 20px;
    }
    #distances {
      font-size: 1.5em;
      margin-top: 20px;
    }
    p {
      margin: 10px 0;
    }
  </style>
</head>
<body>
  <h1>Encoder and Distance Data</h1>
  <div id="distances">
    <p>Encoder A Position: <span id="encoderA">Fetching...</span></p>
    <p>Encoder B Position: <span id="encoderB">Fetching...</span></p>
    <p>Calculated Y Distance: <span id="distanceY">Fetching...</span> cm</p>
  </div>
  <script>
    function updateData() {
      fetch('/distance')
        .then(response => response.json())
        .then(data => {
          document.getElementById('encoderA').innerText = data.encoderA;
          document.getElementById('encoderB').innerText = data.encoderB;
          document.getElementById('distanceY').innerText = data.distanceY.toFixed(2);
        })
        .catch(error => console.error('Error fetching data:', error));
    }
    setInterval(updateData, 500); // Refresh data every 500 ms
  </script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

void handleDistance() {
  float  y = 0;
  float averageEncoder,distanceY;
  aposi=0;bposi=0;
  // Move forward for a fixed duration to calculate distance
  forward();
  delay(500); // Move forward for 500ms
  averageEncoder = (aposi + bposi) / 2.0;
  distance = (wheelCircumference * averageEncoder) / 374.0; // Encoder steps to distance conversion
  y += distanceY;
  aposi=0;bposi=0;
  forward();
  delay(500); 
  averageEncoder = (aposi + bposi) / 2.0;
  distanceY = (wheelCircumference * averageEncoder) / 374.0; // Encoder steps to distance conversion
  y += distanceY;
  aposi=0;bposi=0;
  forward();
  delay(500); 
  averageEncoder = (aposi + bposi) / 2.0;
  distanceY = (wheelCircumference * averageEncoder) / 374.0; // Encoder steps to distance conversion
  y += distanceY;
  aposi=0;bposi=0;
  forward();
  delay(500); 
  averageEncoder = (aposi + bposi) / 2.0;
  distanceY = (wheelCircumference * averageEncoder) / 374.0; // Encoder steps to distance conversion
  y += distanceY;
  aposi=0;bposi=0;
  forward();
  delay(500); 
  averageEncoder = (aposi + bposi) / 2.0;
  distanceY = (wheelCircumference * averageEncoder) / 374.0; // Encoder steps to distance conversion
  y += distanceY;
  aposi=0;bposi=0;
  forward();
  delay(500); 
  averageEncoder = (aposi + bposi) / 2.0;
  distanceY = (wheelCircumference * averageEncoder) / 374.0; // Encoder steps to distance conversion
  y += distanceY;
  aposi=0;bposi=0;
  // Calculate distance traveled based on encoder positions
  

  // Construct JSON response
  String jsonResponse = "{";
  jsonResponse += "\"encoderA\": " + String(aposi) + ",";
  jsonResponse += "\"encoderB\": " + String(bposi) + ",";
  jsonResponse += "\"distanceY\": " + String(y, 2);
  jsonResponse += "}";

  server.send(200, "application/json", jsonResponse);

  // Stop the robot after calculating distance
  stop();
  delay(50000);
}

void loop(){
  server.handleClient();
} 



