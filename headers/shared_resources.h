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

#define TOLERANCE 1E-10

#define IMPASSABLE_OBJECT -1000
#define EXIT_CELL -1001
#define EMPTY_CELL -1002

bool origin_uses_auxiliary_data();
bool origin_uses_static_pedestrians();
bool origin_uses_static_exits();
bool are_same_coordinates(Location first, Location second);
double euclidean_distance(Location first, Location second);
float rand_within_limits(float min, float max);
bool probability_test(double probability);
int roulette_wheel_selection(double *probability_list, int length, double total_probability);

#endif