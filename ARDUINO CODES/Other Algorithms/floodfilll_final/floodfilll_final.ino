#include <LinkedList.h>  // Include the LinkedList library

LinkedList<int> s;  // Declare the linked list as a stack

int c = 0, k, l, x, y;
int a[8][9] = {
  {-1, 1, 1, 1, 1, 1, 1, 1, 0},
  {1, 1, 0, 1, 1, 1, 1, 1, 1},
  {1, 1, 0, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 0, 1},
  {1, 1, 0, 1, 1, 1, 1, 1, 1},
  {1, 1, 1, 1, 1, 1, 1, 0, 1},
  {1, 1, 1, 1, 1, 1, 1, 0, 1},
  {1, 1, 1, 1, 1, 1, 1, 0, -5}
};

int b[8][9];  // Distance matrix
int m = 8;  // Rows
int n = 9;  // Columns

int minimum = 600;
int stackdata;
int ex, ey, ec;

void setup() {
  Serial.begin(9600);  // Initialize serial communication

  // Initialize the distance matrix 'b'
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      if (a[i][j] <= 0) {
        b[i][j] = a[i][j];  // Obstacles and start/end points
      }
      if (a[i][j] == 1) {
        b[i][j] = 100;  // Unvisited open cells
      }
    }
  }

  // Apply BFS-like approach to update the distance matrix 'b'
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      if (b[i][j] == -1) {  // Start point (marked -1)
        adjacent(i, j);
      }
    }
  }

  // Propagate distances across the grid
  for (c = 1; c < 72; c++) {
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < n; j++) {
        if (b[i][j] == c) {
          adjacent(i, j);
        }
      }
    }
  }

  // Find the end point (-5) and push its coordinates to the stack
  for (int i = 0; i < m; i++) {
    for (int j = 0; j < n; j++) {
      if (b[i][j] == -5) {
        ex = i;
        ey = j;
        ec = ex * 100 + ey;
        s.add(ec);  // Add to linked list (stack)
      }
    }
  }

  // Trace the shortest path from the end point back to the start point
  while (minimum != -1) {
    shortestpath(ex, ey);
    ex = stackdata / 100;
    ey = stackdata % 100;
  }

  // Pop and print each element in the stack (the traced path)
  while (s.size() > 0) {
    int pathCoord = s.remove(s.size() - 1);  // Remove from linked list and get the value
    Serial.println(pathCoord);  // Print the coordinates from the stack
  }
}

void loop() {
  // Main loop (empty as we do not need continuous logic here)
}

void adjacent(int x, int y) {
  if (x < (m - 1) && y < n) {
    if (b[x + 1][y] > c) {
      b[x + 1][y] = c + 1;
    }
  }
  if (x < m && y < (n - 1)) {
    if (b[x][y + 1] > c) {
      b[x][y + 1] = c + 1;
    }
  }
  if (x > 0 && y > -1) {
    if (b[x - 1][y] > c) {
      b[x - 1][y] = c + 1;
    }
  }
  if (x > -1 && y > 0) {
    if (b[x][y - 1] > c) {
      b[x][y - 1] = c + 1;
    }
  }
}

int shortestpath(int x, int y) {
  minimum = 600;  // Reset minimum for each step
  int c = 0, d = 0;

  // Check down
  if (x < (m - 1) && y < n) {
    if (b[x + 1][y] != 0) {
      minimum = b[x + 1][y];
      c = x + 1;
      d = y;
    }
  }

  // Check up
  if (x > 0 && y > -1) {
    if ((b[x - 1][y] < minimum) && (b[x - 1][y] != 0)) {
      minimum = b[x - 1][y];
      c = x - 1;
      d = y;
    }
  }

  // Check right
  if (x < m && y < (n - 1)) {
    if ((b[x][y + 1] < minimum) && (b[x][y + 1] != 0)) {
      minimum = b[x][y + 1];
      c = x;
      d = y + 1;
    }
  }

  // Check left
  if (x > -1 && y > 0) {
    if ((b[x][y - 1] < minimum) && (b[x][y - 1] != 0)) {
      minimum = b[x][y - 1];
      c = x;
      d = y - 1;                                                                                                                                     
    }
  }

  stackdata = 100 * c + d;  // Store new coordinates as a single integer
  s.add(stackdata);  // Add to linked list (stack)

  // If we are at the minimum distance, remove the last two elements
  if (b[x][y] == minimum) {
    s.remove(s.size() - 1);  // Remove last element
    s.remove(s.size() - 1);  // Remove previous element
  }

  b[x][y] = 500;  // Mark the current position as visited
  return 0;
}
