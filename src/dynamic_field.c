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
#include"../headers/cell.h"
#include"../headers/pedestrian.h"
#include"../headers/cli_processing.h"
#include"../headers/shared_resources.h"

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
 * Evaluates the decay process for particles within each cell of the dynamic floor field.
 *
 * This function systematically examines the particles located in each cell of the dynamic floor field 
 * to determine whether each particle undergoes decay (i.e., ceases to exist). If decay occurs to a particle,
 * the number of particles in the corresponding cell is decremented by one.
 */
void decay()
{
    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {

            int num_of_particles = exits_set.dynamic_floor_field[i][j];
            for(int particle = 0; particle < num_of_particles; particle++)
            {
                if(probability_test(cli_args.delta))
                {
                    exits_set.dynamic_floor_field[i][j] -= 1;
                }
            }
        }
    }
}

/**
 * Evaluates the diffusion process for particles within each cell of the dynamic floor field.
 * 
 * This function systematically examines the particles located in each cell of the dynamic floor field 
 * to determine whether each particle undergoes diffusion (i.e., particles are created in the horizontal or vertical vicinity).
 * If diffusion occurs to a particle in relation to one of its neighbors, them the corresponding neighbor particle number is incremented by one.
 * 
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
 */
Function_Status diffusion()
{
    Location modifiers[] = {{-1,0}, {0,-1}, {0,1}, {1,0}}; // Diffusion doesn't occur in the diagonals.

    Int_Grid aux = allocate_integer_grid(cli_args.global_line_number, cli_args.global_column_number);
    if(aux == NULL)
        return FAILURE;

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            int num_of_particles = exits_set.dynamic_floor_field[i][j];
            for(int particle = 0; particle < num_of_particles; particle++)
            {
                for(int m = 0; m < 4; m++)
                {
                    if(! is_within_grid_lines(i + modifiers[m].lin) || is_within_grid_columns(j + modifiers[m].col))
                        continue;
                    
                    if(exits_set.static_floor_field[i + modifiers[m].lin][j + modifiers[m].col] == WALL_CELL)
                        continue; // The static floor field shows where the exits are

                    if(probability_test(cli_args.alpha))
                    {
                        aux[i + modifiers[m].lin][j + modifiers[m].col] += 1;
                    }
                }
            }
        }
    }

    if( sum_grids(exits_set.dynamic_floor_field, aux) == FAILURE)
        return FAILURE;

    deallocate_grid((void **) aux, cli_args.global_line_number);

    return SUCCESS;
} 