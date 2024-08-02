#ifndef EXIT_H
#define EXIT_H

#include"shared_resources.h"
#include"grid.h"

struct exit {
    int width; // in contiguous cells
    Location *coordinates; // cells that form up the exit
    Double_Grid static_weight;
    Double_Grid dynamic_weight;
    Double_Grid floor_field;
};
typedef struct exit * Exit;

typedef struct{
    Double_Grid static_floor_field;
    Double_Grid dynamic_floor_field;


    Double_Grid final_floor_field; // Floor field obtained by combining the floor fields of each door
    Exit *list;
    int num_exits;
} Exits_Set;

Function_Status add_new_exit(Location exit_coordinates);
Function_Status expand_exit(Exit original_exit, Location new_coordinates);
Function_Status allocate_final_floor_field();
Function_Status calculate_kirchner_static_field();
Function_Status calculate_all_static_weights();
Function_Status calculate_all_dynamic_weights();
Function_Status calculate_all_exits_floor_field();
Function_Status calculate_final_floor_field();
void deallocate_exits();

extern Exits_Set exits_set;

#endif