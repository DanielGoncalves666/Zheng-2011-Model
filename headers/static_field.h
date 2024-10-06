#ifndef STATIC_FIELD_H
#define STATIC_FIELD_H

#include"shared_resources.h"

void calculate_kirchner_static_field(Location *exit_cell_coordinates, int num_exit_cells, Double_Grid destination_grid);
void calculate_zheng_static_field(Location *exit_cell_coordinates, int num_exit_cells, Double_Grid destination_grid);
Function_Status calculate_all_static_weights();

#endif