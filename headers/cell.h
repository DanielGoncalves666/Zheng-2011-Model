#ifndef CELL_H
#define CELL_H

#include<stdbool.h>

#include"shared_resources.h"

typedef struct{
    Location coordinates;
    double value;
}Cell;

int count_cells_with_smaller_value(Cell *cell_list, int list_length, double searched_value, int *equal_quantity);
void quick_sort(Cell *cell_list, int start_index, int end_index);

#endif