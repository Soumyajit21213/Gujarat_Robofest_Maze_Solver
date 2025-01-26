#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#define MAX_SIZE 5
#define OFFSET 0
//*****************************tremaux**************************************
typedef struct Position {
     int x;
     int y;
    uint8_t direction;
} Position;

typedef struct MazeWall {
    //uint8_t walls; 
    uint8_t travelled[4];   // Wall state (0-255) 0:N 1:E 2:S 3:W
} MazeWall;

typedef struct {
    int x;
    int y;
} Pair;

Pair make_pair(int x, int y) {
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
    for (uint8_t i = 0; i < map->size; ++i) {
        if (pair_equals(map->data[i].coord, key)) {
            return &map->data[i].value;
        }
    }
    return NULL; // Not found
}

// Retrieve the last inserted element (topmost)
MazeWall* get_top(ManualMap* map) {
    if (map->size == 0) {
        return NULL; // Map is empty
    }
    return &map->data[map->size - 1].value; // Return the last inserted element
}

void delete_node(ManualMap* map, Pair key) {
    for (uint8_t i = 0; i < map->size; ++i) {
        if (pair_equals(map->data[i].coord, key)) {
            // Shift elements to fill the gap
            for (uint8_t j = i; j < map->size - 1; ++j) {
                map->data[j] = map->data[j + 1];
            }
            map->size--;
            return;
        }
    }
    printf("Key not found in map!\n");
}

void delete_top(ManualMap* map) {map->size--;}