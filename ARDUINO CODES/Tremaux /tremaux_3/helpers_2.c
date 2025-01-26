#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#define MAX_SIZE 31
#define OFFSET 0
//*****************************tremaux**************************************
typedef struct Position {
    uint8_t x;
    uint8_t y;
    uint8_t direction;
} Position;

typedef struct MazeWall {
    uint8_t walls;    // Wall state (0-255)
    uint8_t visited;  // Visited count (0-255)
} MazeWall;

typedef struct {
    uint8_t x;
    uint8_t y;
} Pair;

Pair make_pair(uint8_t x, uint8_t y) {
    Pair p;
    p.x = x;
    p.y = y;
    return p;
}

// Compare two pairs
bool pair_equals(Pair a, Pair b) {
    return (a.x == b.x && a.y == b.y);
}

// Define Map Entry structure
typedef struct {
    Pair coord;
    MazeWall value;
} MapEntry;

typedef struct {
    MapEntry data[MAX_SIZE];
    uint8_t size;
} ManualMap;

// Insert or update a key-value pair
void insert(ManualMap* map, Pair key, MazeWall value) {
    key.x += OFFSET; // Adjust for negative coordinates
    key.y += OFFSET;
    for (uint8_t i = 0; i < map->size; ++i) {
        if (pair_equals(map->data[i].coord, key)) {
            map->data[i].value = value; // Update if key exists
            return;
        }
    }
    if (map->size < MAX_SIZE) {
        map->data[map->size].coord = key;
        map->data[map->size].value = value;
        map->size++;
    } else {
        printf("Map is full!\n");
    }
}

// Retrieve a value by key
MazeWall* get(ManualMap* map, Pair key) {
    key.x += OFFSET; // Adjust for negative coordinates
    key.y += OFFSET;
    for (uint8_t i = 0; i < map->size; ++i) {
        if (pair_equals(map->data[i].coord, key)) {
            return &map->data[i].value;
        }
    }
    return NULL; // Not found
}

// Display the contents of the map


//**********************************FLoodfill**********************************








