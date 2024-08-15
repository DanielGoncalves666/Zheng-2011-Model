#ifndef EXIT_H
#define EXIT_H

#include"shared_resources.h"
#include"grid.h"

struct exit {
    int width; // in contiguous cells
    Location *coordinates; // cells that form up the exit
    Double_Grid static_weight;
};
typedef struct exit * Exit;

typedef struct{
    Double_Grid static_floor_field;
    Int_Grid dynamic_floor_field;
    Exit *list;
    int num_exits;
} Exits_Set;

Function_Status add_new_exit(Location exit_coordinates);
Function_Status expand_exit(Exit original_exit, Location new_coordinates);
Function_Status allocate_exits_set_fields();
Function_Status calculate_kirchner_static_field();
Function_Status calculate_all_static_weights();
void deallocate_exits();

extern Exits_Set exits_set;

#endif