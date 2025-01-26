#include <WiFi.h>
#include <WebServer.h>
#include <StackArray.h>

StackArray<int> s;
int c = 0, k, l, x, y;
int a[8][9] = {
  {-1, 1, 1, 1, 1, 1, 1, 1, 0},
  {1, 1, 0, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 0, 1},
  {1, 1, 0, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 0, 1},
  {1, 1, 1, 1, 1, 1, 1, 0, 1},
  {1, 1, 1, 1, 1, 1, 1, 0, 1},
  {1, 1, 1, 1, 1, 1, 1, 0, -5}
};
int b[8][9];
int m = 8;
int n = 9;
int ex, ey, ec, stackdata;

WebServer server(80);

// Replace with your network credentials
const char* ssid = "ESP32-Network";
const char* password = "password123";

void handleRoot() {
  String html = "<!DOCTYPE html><html><body><h1>Matrix b</h1><pre>";

  // Add the b matrix to the HTML page
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      html += String(b[i][j]) + "\t";
    }
    html += "\n";
  }

  html += "</pre></body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(9600);
  s.setPrinter(Serial);

  // Initialize Wi-Fi
  WiFi.softAP(ssid, password);
  Serial.println("Wi-Fi AP started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Start server
  server.on("/", handleRoot);
  server.begin();
  Serial.println("Web server started");

  // Initialize matrix b
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      if (a[i][j] <= 0)
        b[i][j] = a[i][j];
      else
        b[i][j] = 100;  // High value for unvisited cells
    }
  }

  // Flood-fill the matrix
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      if (b[i][j] == -1)
        adjacent(i, j);
    }
  }

  for (c = 1; c < 72; c++) {
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < n; j++) {
        if (b[i][j] == c)
          adjacent(i, j);
      }
    }
  }

  // Start point for backtracking
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      if (b[i][j] == -5) {
        ex = i;
        ey = j;
        ec = ex * 100 + ey;
        s.push(ec);
      }
    }
  }

  // Backtrack and fix the b matrix for final output
  while (!s.isEmpty()) {
    stackdata = s.pop();
    int x = stackdata / 100;
    int y = stackdata % 100;

    if (b[x][y] != -1 && b[x][y] != -5) {
      b[x][y] = c--;
    }
  }

  // Serial print the b matrix
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      Serial.print(b[i][j]);
      Serial.print("\t");
    }
    Serial.println();
  }
}

void loop() {
  server.handleClient();  // Handle incoming client requests
}

int adjacent(int x, int y) {
  if (x < (m - 1) && y < n && b[x + 1][y] > c)
    b[x + 1][y] = c + 1;
  if (x < m && y < (n - 1) && b[x][y + 1] > c)
    b[x][y + 1] = c + 1;
  if (x > 0 && y > -1 && b[x - 1][y] > c)
    b[x - 1][y] = c + 1;
  if (x > -1 && y > 0 && b[x][y - 1] > c)
    b[x][y - 1] = c + 1;
  return 0;
}