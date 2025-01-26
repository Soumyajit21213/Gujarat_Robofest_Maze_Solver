#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define MAX_SIZE 100
#define MAX_PATHS 4  // Maximum number of possible paths (N, E, S, W)

//*******************************************************Coordinates***************************************************************

// Define Pair structure for coordinates
typedef struct {
    int x;
    int y;
} Pair;

Pair make_pair(uint8_t x, uint8_t y) {
    return (Pair){x, y};
}

// Compare two pairs
bool pair_equals(Pair a, Pair b) {
    return (a.x == b.x && a.y == b.y);
}

//*******************************************************Node Structure***************************************************************

// Define Node structure
typedef struct {
    Pair coord;            // Coordinate point
    uint8_t visited_count; // Number of visits
    uint8_t dir;           // Current direction (0: North, 1: East, 2: South, 3: West)
    char open[MAX_PATHS];  // Store open paths ('r', 'f', 'l', etc.)
    uint8_t open_count[MAX_PATHS]; // Number of times each open path has been taken
} Node;

//*******************************************************Stack Operations***************************************************************

// Define Stack structure
typedef struct {
    Node* nodes[MAX_SIZE];  // Array of node pointers
    size_t top;             // Index of the top element in the stack
} Stack;

// Initialize the stack
void init_stack(Stack* stack) {
    stack->top = 0;
}

// Push a node onto the stack
bool push(Stack* stack, Node* node) {
    if (stack->top < MAX_SIZE) {
        stack->nodes[stack->top++] = node;
        return true;
    } else {
        printf("Stack Overflow!\n");
        return false;
    }
}

// Pop a node from the stack
Node* pop(Stack* stack) {
    if (stack->top > 0) {
        return stack->nodes[--stack->top];
    } else {
        printf("Stack Underflow!\n");
        return NULL;
    }
}

// Peek at the top element of the stack
Node* peek(Stack* stack) {
    if (stack->top > 0) {
        return stack->nodes[stack->top - 1];
    } else {
        return NULL;
    }
}

// Free the stack's memory
void free_stack(Stack* stack) {
    while (stack->top > 0) {
        Node* node = pop(stack);
        if (node) free(node);
    }
}

//******************************************************Node Operations***************************************************************

// Create a new node for the stack
Node* create_node(Pair coord, uint8_t dir, const char open[MAX_PATHS], const uint8_t open_count[MAX_PATHS]) {
    Node* new_node = (Node*)malloc(sizeof(Node));
    if (!new_node) {
        printf("Error: Memory allocation failed for node.\n");
        return NULL;
    }
    new_node->coord = coord;
    new_node->visited_count = 1;
    new_node->dir = dir;
    memcpy(new_node->open, open, sizeof(new_node->open));
    memcpy(new_node->open_count, open_count, sizeof(new_node->open_count));  // Correctly initializing open_count
    return new_node;
}

// Insert or update a node in the stack
bool insert(Stack* stack, Pair coord, uint8_t dir, const char open[MAX_PATHS], const uint8_t open_count[MAX_PATHS]) {
    // Check if the coordinate already exists in the stack
    for (size_t i = 0; i < stack->top; ++i) {
        if (pair_equals(stack->nodes[i]->coord, coord)) {
            // Node already exists, update its properties
            stack->nodes[i]->visited_count++;
            memcpy(stack->nodes[i]->open, open, sizeof(stack->nodes[i]->open));
            
            // Instead of overwriting open_count, increment it
            for (int j = 0; j < MAX_PATHS; j++) {
                stack->nodes[i]->open_count[j] += open_count[j];
            }

            stack->nodes[i]->dir = dir;
            return true;
        }
    }

    // Add a new node
    Node* new_node = create_node(coord, dir, open, open_count);
    if (!new_node) return false;

    return push(stack, new_node);
}

bool update_node(Stack* stack, Pair coord, uint8_t dir, const char open[MAX_PATHS], const uint8_t open_count[MAX_PATHS]) {
    for (size_t i = 0; i < stack->top; ++i) {
        if (pair_equals(stack->nodes[i]->coord, coord)) {
            // Update existing node's properties
            stack->nodes[i]->dir = dir;
            memcpy(stack->nodes[i]->open, open, sizeof(stack->nodes[i]->open));

            // Update open_count (either replace or increment values)
            for (int j = 0; j < MAX_PATHS; j++) {
                stack->nodes[i]->open_count[j] = open_count[j];  // Update with new values
            }

            return true;  // Node updated successfully
        }
    }
    return false;  // Node not found
}


// Retrieve a node by its coordinates
Node* get_node(Stack* stack, Pair coord) {
    for (size_t i = 0; i < stack->top; ++i) {
        if (pair_equals(stack->nodes[i]->coord, coord)) {
            return stack->nodes[i];
        }
    }
    return NULL;  // Node not found
}

// Increment the count for a specific open path
typedef enum { NORTH = 0, EAST, SOUTH, WEST } Direction;
void increment_open_count(Node* node, Direction dir) {
    if (node && dir < MAX_PATHS) {
        node->open_count[dir]++;
    }
}