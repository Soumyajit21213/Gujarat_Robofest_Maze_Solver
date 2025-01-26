#include <WiFi.h>
#include <WebServer.h>

// Ultrasonic sensor pins
const int trigPinLeft = 42;
const int echoPinLeft = 41;

const int trigPinCentre = 40;
const int echoPinCentre = 37;

const int trigPinRight = 36;
const int echoPinRight = 35;

const int trigPinLF = 39;
const int echoPinLF = 19;
const int trigPinRF = 43;
const int echoPinRF = 20;

// Wi-Fi credentials
const char* ssid = "ESP32_Ultrasonic";
const char* password = "12345678";

// Create a web server object
WebServer server(80);

void setup() {
  // Setup ultrasonic sensor pins
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  
  pinMode(trigPinCentre, OUTPUT);
  pinMode(echoPinCentre, INPUT);
  
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
  
  pinMode(trigPinLF, OUTPUT);
  pinMode(echoPinLF, INPUT);

  pinMode(trigPinRF, OUTPUT);
  pinMode(echoPinRF, INPUT);
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

void loop() {
  // Handle incoming web requests
  server.handleClient();
}

// Function to calculate distance from an ultrasonic sensor
float getUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.034) / 2; // Convert to cm
  return distance;
}

void handleRoot() {
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <title>Ultrasonic Distances</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: Arial, sans-serif; text-align: center; padding: 20px; }
      h1 { font-size: 2.5em; margin-bottom: 20px; }
      #distances { font-size: 1.5em; margin-top: 20px; }
      p { margin: 10px 0; }
    </style>
  </head>
  <body>
    <h1>Ultrasonic Sensor Data</h1>
    <div id="distances">
      <p>Distance Left: <span id="distanceLeft">Fetching...</span> cm</p>
      <p>Distance Left-Forward: <span id="distanceLF">Fetching...</span> cm</p>
      <p>Distance Centre: <span id="distanceCentre">Fetching...</span> cm</p>
      <p>Distance Right-Forward: <span id="distanceRF">Fetching...</span> cm</p>
      <p>Distance Right: <span id="distanceRight">Fetching...</span> cm</p>
    </div>
    <script>
      function updateDistances() {
        fetch('/distance')
          .then(response => response.json())
          .then(data => {
            document.getElementById('distanceLeft').innerText = data.left.toFixed(2);
            document.getElementById('distanceLF').innerText = data.leftForward.toFixed(2);
            document.getElementById('distanceCentre').innerText = data.centre.toFixed(2);
            document.getElementById('distanceRF').innerText = data.rightForward.toFixed(2);
            document.getElementById('distanceRight').innerText = data.right.toFixed(2);
          })
          .catch(error => console.error('Error fetching distances:', error));
      }
      setInterval(updateDistances, 500); // Update every 500ms
    </script>
  </body>
  </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleDistance() {
  float distanceLeft = getUltrasonicDistance(trigPinLeft, echoPinLeft);
  float distanceCentre = getUltrasonicDistance(trigPinCentre, echoPinCentre);
  float distanceRight = getUltrasonicDistance(trigPinRight, echoPinRight);
  float distanceLF = getUltrasonicDistance(trigPinLF, echoPinLF);
  float distanceRF = getUltrasonicDistance(trigPinRF, echoPinRF);

  // Create JSON response
  String jsonResponse = "{";
  jsonResponse += "\"left\":" + String(distanceLeft, 2) + ",";
  jsonResponse += "\"leftForward\":" + String(distanceLF, 2) + ",";
  jsonResponse += "\"centre\":" + String(distanceCentre, 2) + ",";
  jsonResponse += "\"rightForward\":" + String(distanceRF, 2) + ",";
  jsonResponse += "\"right\":" + String(distanceRight, 2);
  jsonResponse += "}";

  server.send(200, "application/json", jsonResponse);
}


