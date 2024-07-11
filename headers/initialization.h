#ifndef INITIALIZATION_H
#define INITIALIZATION_H

#include"shared_resources.h"

Function_Status open_auxiliary_file(FILE **auxiliary_file);
Function_Status open_output_file(FILE **output_file);
Function_Status allocate_grids();
Function_Status load_environment();
Function_Status generate_environment();
int extract_simulation_set_quantity(FILE *auxiliary_file);
Function_Status get_next_simulation_set(FILE *auxiliary_file, int *exit_number);

#endif