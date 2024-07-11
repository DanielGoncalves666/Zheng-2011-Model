#ifndef PRINTING_UTILITIES_H
#define PRINTING_UTILITIES_H

#include"grid.h"

void print_full_command(FILE *output_stream);
void print_heatmap(FILE *output_stream);
void print_pedestrian_position_grid(FILE *output_stream, int simulation_number, int timestep);
void print_int_grid(Int_Grid int_grid);
void print_double_grid(Double_Grid double_grid);
void print_simulation_set_information(FILE *output_stream);
void print_execution_status(int set_index, int set_quantity);
void print_placeholder(FILE *stream, int placeholder);

#endif