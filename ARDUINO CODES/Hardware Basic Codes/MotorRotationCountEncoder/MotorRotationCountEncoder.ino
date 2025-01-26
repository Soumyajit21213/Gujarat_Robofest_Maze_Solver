#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "ESP32_Encoder"; // SSID of your Wi-Fi access point
const char* password = "12345678"; // Wi-Fi password

// Create a WebServer object on port 80
WebServer server(80);

const int encoderA1 = 5;
const int encoderA2 = 6;
const int encoderB1 = 12;
const int encoderB2 = 11;

long aposi = 0;
long bposi = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/


void setup() {
  Serial.begin(115200); // Increased baud rate for faster serial communication
  pinMode(encoderA1, INPUT); // Set pin mode for encoderA1 (encoder A)
  pinMode(encoderA2, INPUT); // Set pin mode for encoderA2 (encoder B)
  pinMode(encoderB1, INPUT); // Set pin mode for encoderA1 (encoder A)
  pinMode(encoderB2, INPUT); // Set pin mode for encoderA2 (encoder B)
  attachInterrupt(digitalPinToInterrupt(encoderA1), handleEncoderA, RISING); 
  attachInterrupt(digitalPinToInterrupt(encoderB1), handleEncoderB, RISING); // Attach interrupt to encoderA1 (encoder A)

  // Start Wi-Fi in AP (Access Point) mode
  WiFi.softAP(ssid, password);
  Serial.println("WiFi AP Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP()); // Print the IP address of the ESP32

  // Define HTTP routes
  server.on("/", handleRoot); // Handle the root page
  server.on("/encoder", handleEncoder); // Handle encoder value requests

  // Start the web server
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  // Print the encoder position in the Serial Monitor
  int apos = aposi;
  int bpos = bposi; // Read the encoder position
  

  // Handle client requests
  server.handleClient();
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

void handleRoot() {
  // Serve the HTML page that shows the encoder value
  String html = "<!DOCTYPE html>\n<html>\n<head>\n<title>Encoder Value</title>\n";
  html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n";
  html += "<style>\nbody { font-family: Arial, sans-serif; text-align: center; padding: 20px; }\n";
  html += "h1 { font-size: 2.5em; margin-bottom: 20px; }\n";
  html += "#encoder { font-size: 2em; margin-top: 20px; }\n</style>\n</head>\n<body>\n";
  html += "<h1>Encoder Values</h1>\n";
  html += "<div id=\"encoder\">\n";
  html += "  <p>A Encoder Value: <span id=\"Apos\">Fetching...</span></p>\n";
  html += "  <p>B Encoder Value: <span id=\"Bpos\">Fetching...</span></p>\n";
  html += "</div>\n";
  html += "<script>\n";
  html += "function updateEncoder() {\n";
  html += "  fetch('/encoder')\n";
  html += "    .then(response => response.json())\n";
  html += "    .then(data => {\n";
  html += "      document.getElementById('Apos').innerText = data.A_value;\n";
  html += "      document.getElementById('Bpos').innerText = data.B_value;\n";
  html += "    });\n";
  html += "}\n";
  html += "setInterval(updateEncoder, 100);\n"; // Update the encoder value every 100 ms
  html += "</script>\n";
  html += "</body>\n</html>";

  server.send(200, "text/html", html); // Send the HTML page to the client
}

void handleEncoder() {
  // Serve the encoder values in JSON format
  String jsonResponse = "{";
  jsonResponse += "\"A_value\":" + String(aposi) + ",";
  jsonResponse += "\"B_value\":" + String(bposi);
  jsonResponse += "}";

  server.send(200, "application/json", jsonResponse); // Send the JSON response
}