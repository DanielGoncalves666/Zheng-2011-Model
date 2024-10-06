/* 
   File: fire_Field.c
   Author: Daniel Gonçalves
   Date: 15/09/2024
   Description: 
*/

#include<stdlib.h>

#include"../headers/fire_dynamics.h"
#include"../headers/grid.h"
#include"../headers/cli_processing.h"
#include"../headers/shared_resources.h"

Location moore_modifiers[8] = {{-1,-1}, {-1,0}, {-1,1}, {0,-1}, {0,1}, {1,-1}, {1,0}, {1,1}};

Int_Grid fire_grid = NULL; // Grid containing only fire.
                           // Contains cells with wither FIRE_CELL or EMPTY_CELL values
Int_Grid initial_fire_grid = NULL; // Grid just like the fire_grid, but holds the location of the initial fires. Once initialized never changes.

/**
 * Propagate the fire in the environment, in accordance with the Zheng's 2011 article.
 */
void zheng_fire_propagation()
{
    Int_Grid aux = allocate_integer_grid(cli_args.global_line_number, cli_args.global_column_number);
    fill_integer_grid(aux, cli_args.global_line_number, cli_args.global_column_number, EMPTY_CELL);

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            if(! is_cell_with_fire((Location) {i,j}))
                continue;

            aux[i][j] = FIRE_CELL;

            for(int mm = 0; mm < 8; mm++)
            {
                if( ! is_within_grid_lines(i + moore_modifiers[mm].lin) || 
                    ! is_within_grid_columns(j + moore_modifiers[mm].col) ||
                    obstacle_grid[i + moore_modifiers[mm].lin][j + moore_modifiers[mm].col] != EMPTY_CELL)
                    continue;

                aux[i + moore_modifiers[mm].lin][j + moore_modifiers[mm].col] = FIRE_CELL;
            }
        }
    }
    
    copy_integer_grid(fire_grid, aux);
    deallocate_grid((void **) aux, cli_args.global_line_number);
}