#ifndef PEDESTRIAN_H
#define PEDESTRIAN_H

#include"shared_resources.h"

typedef struct cell_conflict * Cell_Conflict;

enum Pedestrian_State {LEAVING, GOT_OUT, STOPPED, MOVING};

struct pedestrian {
    int id;
    enum Pedestrian_State state;
    Location origin; // Original pedestrian localization. Remains unchanged until the structure instance is deallocated.
    Location previous;
    Location current; 
    Location target;
    double probabilities[3][3];
};
typedef struct pedestrian * Pedestrian;

typedef struct{
    Pedestrian *list;
    int num_pedestrians;
} Pedestrian_Set;

Function_Status insert_pedestrians_at_random(int qtd);
Function_Status add_new_pedestrian(Location pedestrian_coordinates);
void deallocate_pedestrians();
void evaluate_pedestrians_movements();
Function_Status identify_pedestrian_conflicts(Cell_Conflict *pedestrian_conflicts, int *num_conflicts);
Function_Status solve_pedestrian_conflicts(Cell_Conflict pedestrian_conflicts, int num_conflicts);
void print_pedestrian_conflict_information(Cell_Conflict pedestrian_conflicts, int num_conflicts);
void block_X_movement();
void apply_pedestrian_movement();
void update_pedestrian_position_grid();
bool is_environment_empty();
void reset_pedestrian_state();
void reset_pedestrians_structures();

extern Pedestrian_Set pedestrian_set;

#endif