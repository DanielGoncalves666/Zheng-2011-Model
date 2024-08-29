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
#include"../headers/printing_utilities.h"

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
 * If diffusion is triggered, a single particle is created in a valid neighboring cell (the chosen neighboring particle count is incremented by one).
 * 
 * @param is_moving Indicates whether diffusion moves the particle from its origin (true) or simply creates a new particle in the vicinity (false). The value True generates results close to the ones the author has obtained.
 * 
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
 */
Function_Status single_diffusion(bool is_moving)
{
    static Location modifiers[] = {{-1,0}, {0,-1}, {0,1}, {1,0}}; // Diffusion doesn't occur in the diagonals.
    static double probabilities[] = {1, 1, 1, 1};

    if( fill_integer_grid(aux_dynamic_grid, cli_args.global_line_number, cli_args.global_column_number, 0) == FAILURE)
        return FAILURE;
    // Reset to 0 all positions of the auxiliary grid.

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            int num_of_particles = exits_set.dynamic_floor_field[i][j];
            for(int particle = 0; particle < num_of_particles; particle++)
            {
                if(! probability_test(cli_args.alpha))
                    continue; // The event doesn't happen.

                int valid_neighbors = 4;
                char valid_byte = 0; // The 4 least significant bits identify which cells are valid (1) or not (0) for diffusion.
                for(int m = 0; m < 4; m++)
                {
                    if(! is_within_grid_lines(i + modifiers[m].lin) || ! is_within_grid_columns(j + modifiers[m].col))
                    {
                        valid_neighbors--;
                        continue;
                    }
                    
                    if(exits_set.static_floor_field[i + modifiers[m].lin][j + modifiers[m].col] == WALL_CELL)
                    {
                        valid_neighbors--;
                        continue; // The static floor field shows where the exits are, so they can still receive particles.
                    }

                    valid_byte ^= 1U << (3 - m);
                }

                int chosen_index = roulette_wheel_selection((double *) &probabilities, valid_neighbors, valid_neighbors);

                for(int m = 0; m < 4; m++)
                {
                    if((valid_byte & (1U << (3 - m))) != 0)
                        chosen_index--;

                    if(chosen_index == -1)
                    {
                        if(is_moving)
                            aux_dynamic_grid[i][j] -= 1; // Diffusion that moves the particles.

                        aux_dynamic_grid[i + modifiers[m].lin][j + modifiers[m].col] += 1;
                        break;
                    }
                }
            }
        }
    }

    if( sum_grids(exits_set.dynamic_floor_field, aux_dynamic_grid) == FAILURE)
        return FAILURE;

    return SUCCESS;
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
Function_Status multiple_diffusion()
{
    static Location modifiers[] = {{-1,0}, {0,-1}, {0,1}, {1,0}}; // Diffusion doesn't occur in the diagonals.
    
    if( fill_integer_grid(aux_dynamic_grid, cli_args.global_line_number, cli_args.global_column_number, 0) == FAILURE)
        return FAILURE;
    // Reset to 0 all positions of the auxiliary grid.

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            int num_of_particles = exits_set.dynamic_floor_field[i][j];
            for(int particle = 0; particle < num_of_particles; particle++)
            {
                for(int m = 0; m < 4; m++)
                {
                    if(! is_within_grid_lines(i + modifiers[m].lin) || ! is_within_grid_columns(j + modifiers[m].col))
                        continue;
                    
                    if(exits_set.static_floor_field[i + modifiers[m].lin][j + modifiers[m].col] == WALL_CELL)
                        continue; // The static floor field shows where the exits are

                    if(probability_test(cli_args.alpha))
                    {
                        aux_dynamic_grid[i + modifiers[m].lin][j + modifiers[m].col] += 1;
                    }
                }
            }
        }
    }
    
    if( sum_grids(exits_set.dynamic_floor_field, aux_dynamic_grid) == FAILURE)
        return FAILURE;
    
    return SUCCESS;
} 