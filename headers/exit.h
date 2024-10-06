#ifndef EXIT_H
#define EXIT_H

#include"shared_resources.h"
#include"grid.h"

struct exit {
    int width; // in contiguous cells
    bool is_blocked_by_fire;
    Location *coordinates; // cells that form up the exit
    Int_Grid private_structure_grid; // Grid containing obstacles and exit cells. Once initialized, remains unchanged.
    Double_Grid varas_static_weight; // Grid containing the calculation of the static floor field based upon the private_structure_grid
};
typedef struct exit * Exit;

typedef struct{
    Double_Grid static_floor_field;
    Double_Grid dynamic_floor_field;
    Double_Grid fire_floor_field;
    Exit *list;
    int num_exits;
    Double_Grid distance_to_exits_grid; // Grid storing the distance to the nearest exit for each cell.
    Double_Grid aux_static_grid; // Temporary auxiliary grid for storing an alternative static floor field, used for pedestrians unable to visualize certain exits.
    Double_Grid aux_dynamic_grid; // Grid used to help in the diffusion process.
} Exits_Set;

Function_Status add_new_exit(Location exit_coordinates);
Function_Status expand_exit(Exit original_exit, Location new_coordinates);
Function_Status set_private_grid_data(Exit current_exit);
Function_Status allocate_exits_set_fields();
void deallocate_exits();
void check_for_exits_blocked_by_fire();
Location *extract_non_blocked_exit_coordinates(int *num_exit_cells);
void calculate_distance_to_closest_exit(Location *exit_cell_coordinates, int num_exit_cells);
void reset_exits();
bool is_exit_accessible(Exit current_exit);

extern Int_Grid exits_only_grid;
extern Exits_Set exits_set;
extern Location non_diagonal_modifiers[4];

#endif