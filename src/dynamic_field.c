/* 
   File: dynamic_field.c
   Author: Daniel Gonçalves
   Date: 2024-08-15
   Description: This module contains functions related to the Kirchner dynamic floor field.
*/

#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>

#include"../headers/exit.h"
#include"../headers/grid.h"
#include"../headers/pedestrian.h"
#include"../headers/cli_processing.h"
#include"../headers/shared_resources.h"

static void normalize_dynamic_field_values(Double_Grid to_be_normalized, double total_sum);

/**
 * Increases the cell at the given coordinates in one particle.
 * 
 * @param coordinates The coordinates of the cell which value will be increased.
 */
void increase_particle_at(Location coordinates)
{
    exits_set.dynamic_floor_field[coordinates.lin][coordinates.col] += 1;
}

/**
 * Evaluates the decay and diffusion for all cells in the dynamic floor field, normalizing them after the process is completed.
 * 
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
 */
Function_Status apply_decay_and_diffusion()
{
    static Location modifiers[] = {{-1,0}, {0,-1}, {0,1}, {1,0}}; // Diffusion doesn't occur in the diagonals.

    if( fill_double_grid(exits_set.aux_dynamic_grid, cli_args.global_line_number, cli_args.global_column_number, 0) == FAILURE)
        return FAILURE;

    double total_sum = 0;
    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            if(exits_set.static_floor_field[i][j] == IMPASSABLE_OBJECT || is_cell_with_fire((Location) {i,j}))
                continue;

            exits_set.aux_dynamic_grid[i][j] = (1 - cli_args.alpha) * (1 - cli_args.delta) * exits_set.dynamic_floor_field[i][j];

            double neighbor_sum = 0;
            for(int m = 0; m < 4; m++)
            {
                if(! is_within_grid_lines(i + modifiers[m].lin) || ! is_within_grid_columns(j + modifiers[m].col))
                    continue;

                if(exits_set.static_floor_field[i + modifiers[m].lin][j + modifiers[m].col] == IMPASSABLE_OBJECT || 
                    is_cell_with_fire((Location) {i + modifiers[m].lin, j + modifiers[m].col}))
                    continue;

                neighbor_sum += exits_set.dynamic_floor_field[i + modifiers[m].lin][j + modifiers[m].col];
            }

            exits_set.aux_dynamic_grid[i][j] += cli_args.alpha * ((1 - cli_args.delta) / 4) * neighbor_sum;
            total_sum += exits_set.aux_dynamic_grid[i][j];
        }
    }

    normalize_dynamic_field_values(exits_set.aux_dynamic_grid, total_sum);

    if( copy_double_grid(exits_set.dynamic_floor_field, exits_set.aux_dynamic_grid) == FAILURE)
        return FAILURE;

    return SUCCESS;
}

/* ---------------- ---------------- ---------------- ---------------- ---------------- */
/* ---------------- ---------------- STATIC FUNCTIONS ---------------- ---------------- */
/* ---------------- ---------------- ---------------- ---------------- ---------------- */

/**
 * Normalizes all values of the given Double_grid. The normalization for each position is the position value divided by the total_sum provided.
 * 
 * @param to_be_normalized Double grid whose values will be normalized
 * @param total_sum The sum of all values in the provided grid.
 * 
 * @note If total_sum is equal to 0, nothing is done.
 */
static void normalize_dynamic_field_values(Double_Grid to_be_normalized, double total_sum)
{
    if(total_sum == 0)
        return;

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            to_be_normalized[i][j] /= total_sum;
        }
    }
}
