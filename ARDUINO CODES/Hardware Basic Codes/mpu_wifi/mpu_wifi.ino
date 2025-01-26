#include <Wire.h>
#include <MPU6050.h>

#include <WiFi.h>
#include <WebServer.h>
// Wi-Fi credentials
const char* ssid = "ESP32_Ultrasonic";
const char* password = "12345678";

// Create a web server object
WebServer server(80);
MPU6050 accelgyro;

int motionStatus = 0; // 0: stationary, 1: moving
unsigned long stationaryStartTime = 0; // Track when stationary started
const unsigned long stationaryThreshold = 1000; // 3 seconds in milliseconds

void setup() {
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
    Wire.begin(45,0);
    Serial.begin(38400);
    accelgyro.initialize();

    if (!accelgyro.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }

    // Configure motion detection
    accelgyro.setMotionDetectionThreshold(1); // Adjust as needed
    accelgyro.setMotionDetectionDuration(2); // Adjust as needed
    accelgyro.setIntMotionEnabled(true);

    // Serial.println("Motion detection enabled!"); // Commented out
}

void loop() {
  server.handleClient();
    bool motionDetected = accelgyro.getIntMotionStatus();
    unsigned long currentTime = millis();

    if (motionDetected) {
        motionStatus = 1; // Moving
        stationaryStartTime = currentTime; // Reset stationary timer
    } else {
        // Check if stationary for more than the threshold
        if (currentTime - stationaryStartTime > stationaryThreshold) {
            motionStatus = 0; // Stationary for more than 3 seconds
        }
    }

    Serial.println(motionStatus); // Print only motion status
    delay(200);
}
void handleRoot() {
  String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <title>Motion Status</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: Arial, sans-serif; text-align: center; padding: 20px; }
      h1 { font-size: 2.5em; margin-bottom: 20px; }
      #status { font-size: 1.5em; margin-top: 20px; }
      p { margin: 10px 0; }
    </style>
  </head>
  <body>
    <h1>MPU6050 Motion Status</h1>
    <div id="status">
      <p>Current Motion Status: <span id="motionStatus">Fetching...</span></p>
    </div>
    <script>
      function updateStatus() {
        fetch('/distance')
          .then(response => response.json())
          .then(data => {
            document.getElementById('motionStatus').innerText = data.motionStatus === 1 ? "Moving" : "Stationary";
          })
          .catch(error => console.error('Error fetching motion status:', error));
      }
      setInterval(updateStatus, 500); // Update every 500ms
    </script>
  </body>
  </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleDistance() {
  // Create JSON response with motion status
  String jsonResponse = "{";
  jsonResponse += "\"motionStatus\":" + String(motionStatus);
  jsonResponse += "}";

  server.send(200, "application/json", jsonResponse);
}
