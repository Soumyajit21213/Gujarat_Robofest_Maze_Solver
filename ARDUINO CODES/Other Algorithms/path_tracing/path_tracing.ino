#include <WiFi.h>
#include <WebServer.h>

// Define the matrix dimensions
const int rows = 8;
const int cols = 9;
int b[rows][cols] = {
    {-1,  1,  2,  3,  4,  5,  6,  7,  0},
    { 1,  2,  0,  4,  5,  6,  7,  8,  9},
    { 2,  3,  4,  5,  6,  7,  8,  0, 10},
    { 3,  4,  0,  6,  7,  8,  9, 10, 11},
    { 4,  5,  6,  7,  8,  9, 10,  0, 12},
    { 5,  6,  7,  8,  9, 10, 11,  0, 13},
    { 6,  7,  8,  9, 10, 11, 12,  0, 14},
    { 7,  8,  9, 10, 11, 12, 13,  0, -5}
};

// Visited matrix for floodfill
bool visited[rows][cols] = {false};

// Path to store coordinates
int pathX[rows * cols];
int pathY[rows * cols];
int pathIndex = 0;

// Web server instance
WebServer server(80);

// Wi-Fi Credentials
const char* ssid = "ESP32-Network";
const char* password = "password123";

// Function to handle the root route
void handleRoot() {
    String html = "<!DOCTYPE html><html><body><h1>Matrix b</h1><pre>";

    // Add the matrix 'b' to the HTML page
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            html += String(b[i][j]) + "\t";
        }
        html += "\n";
    }

    html += "</pre><h1>Floodfill Path</h1><pre>";

    // Add the floodfill path to the HTML page
    for (int i = 0; i < pathIndex; i++) {
        html += "(" + String(pathX[i]) + "," + String(pathY[i]) + ")\t";
    }

    html += "</pre></body></html>";
    server.send(200, "text/html", html);
}

// Floodfill function to find the path
void floodfill(int x, int y) {
    // Boundary checks and avoid revisiting already visited cells
    if (x < 0 || x >= rows || y < 0 || y >= cols || visited[x][y] || b[x][y] == 0) {
        return;
    }

    // If we reach -5, stop and do not continue the recursion
    if (b[x][y] == -5) {
        visited[x][y] = true;
        pathX[pathIndex] = x;
        pathY[pathIndex] = y;
        pathIndex++;
        return; // Stop further recursion
    }

    // Mark the current cell as visited
    visited[x][y] = true;
    pathX[pathIndex] = x;
    pathY[pathIndex] = y;
    pathIndex++;

    // Define possible moves: right, down, left, up
    int moves[4][2] = {
        {0, 1},  // Right
        {1, 0},  // Down
        {0, -1}, // Left
        {-1, 0}  // Up
    };

    // Recursively explore neighboring cells
    for (int i = 0; i < 4; i++) {
        int nx = x + moves[i][0];
        int ny = y + moves[i][1];
        if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && !visited[nx][ny] && b[nx][ny] != 0) {
            floodfill(nx, ny);
        }
    }
}

void setup() {
    Serial.begin(9600);

    // Initialize Wi-Fi
    WiFi.softAP(ssid, password);
    Serial.println("Wi-Fi AP started");
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());

    // Start web server
    server.on("/", handleRoot);
    server.begin();
    Serial.println("Web server started");

    // Perform floodfill from the start position (0, 0)
    int startX = 0, startY = 0;
    floodfill(startX, startY);
}

void loop() {
    server.handleClient();  // Handle incoming client requests
}
