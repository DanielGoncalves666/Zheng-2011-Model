#ifndef SHARED_RESOURCES_H
#define SHARED_RESOURCES_H

#include<stdbool.h>

enum Simulation_Type {
    SIMULATION_DENSITY = 0,
    SIMULATION_ALPHA,
    SIMULATION_DELTA,
    SIMULATION_STATIC_COUPLING,
    SIMULATION_DYNAMIC_COUPLING,
    SIMULATION_DOOR_LOCATION_ONLY
};
// All simulation types support the variation of door location.

enum Output_Format {
    OUTPUT_VISUALIZATION = 1, 
    OUTPUT_TIMESTEPS_COUNT, 
    OUTPUT_HEATMAP,
};

enum Environment_Origin {
    ONLY_STRUCTURE = 1, 
    STRUCTURE_AND_DOORS, 
    STRUCTURE_AND_PEDESTRIANS, 
    STRUCTURE_DOORS_AND_PEDESTRIANS, 
    AUTOMATIC_CREATED
};

typedef enum Function_Status {
    FAILURE = 0, 
    END_PROGRAM = 0,
    SUCCESS = 1, 
    INACCESSIBLE_EXIT = 2
}Function_Status;

typedef struct{
    int lin;
    int col;
}Location;

#define EXIT_VALUE 1
#define WALL_VALUE -1000

bool origin_uses_auxiliary_data();
bool origin_uses_static_pedestrians();
bool origin_uses_static_exits();
double euclidean_distance(Location first, Location second);

#endif