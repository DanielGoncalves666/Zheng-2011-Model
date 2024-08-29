#ifndef GRID_H
#define GRID_H

#include<stdbool.h>

#include"shared_resources.h"

typedef int ** Int_Grid;
typedef double ** Double_Grid;

Int_Grid allocate_integer_grid(int line_number, int column_number);
Double_Grid allocate_double_grid(int line_number, int column_number);
Function_Status fill_integer_grid(Int_Grid integer_grid, int line_number, int column_number, int value);
Function_Status fill_double_grid(Double_Grid double_grid, int line_number, int column_number, double value);
Function_Status copy_double_grid(Double_Grid destination, Double_Grid source);
Function_Status copy_grid_structure(Double_Grid destination, Int_Grid source);
Function_Status sum_grids(Int_Grid destination, Int_Grid source);
bool is_diagonal_valid(Location origin_cell, Location target_cell, Double_Grid floor_field);
bool is_within_grid_lines(int line_coordinate);
bool is_within_grid_columns(int column_coordinate);
bool is_cell_empty(Location coordinates);
void deallocate_grid(void **grid, int line_number);

extern Int_Grid environment_only_grid;
extern Int_Grid exits_only_grid;
extern Int_Grid pedestrian_position_grid;
extern Int_Grid heatmap_grid;
extern Int_Grid aux_dynamic_grid;

#endif