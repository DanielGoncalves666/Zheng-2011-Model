/* 
   File: exit.c
   Author: Daniel Gonçalves
   Date: 2023-10-15
   Description: This module contains declarations of structures to hold exit information and functions to create/expand exits, add exits to the exits set, and calculate the floor field.
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

Exits_Set exits_set = {NULL, NULL, 0};

static Exit create_new_exit(Location exit_coordinates);
static Function_Status calculate_static_weight(Exit current_exit);
static void initialize_static_weight_grid(Exit current_exit);
static bool is_exit_accessible(Exit s);
static int identify_occupied_cells(Cell **occupied_cells, Exit current_exit);

/**
 * Adds a new exit to the exits set.
 * 
 * @param exit_coordinates New exit coordinates.
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
*/
Function_Status add_new_exit(Location exit_coordinates)
{
    Exit new_exit = create_new_exit(exit_coordinates);
    if(new_exit == NULL)
    {
        fprintf(stderr,"Failure on creating an exit at coordinates (%d,%d).\n",exit_coordinates.lin, exit_coordinates.col);
        return FAILURE;
    }

    exits_set.num_exits += 1;
    exits_set.list = realloc(exits_set.list, sizeof(Exit) * exits_set.num_exits);
    if(exits_set.list == NULL)
    {
        fprintf(stderr, "Failure in the realloc of the exits_set list.\n");
        return FAILURE;
    }    

    exits_set.list[exits_set.num_exits - 1] = new_exit;

    return SUCCESS;
}

/**
 * Expands an existing exit by adding a new cell based on the provided coordinates.
 * 
 * @param original_exit Exit to be expanded.
 * @param new_coordinates Coordinates of the cell to be added to the exit. 
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
*/
Function_Status expand_exit(Exit original_exit, Location new_coordinates)
{
    if(is_within_grid_lines(new_coordinates.lin) && is_within_grid_columns(new_coordinates.col))
    {
        original_exit->width += 1;
        original_exit->coordinates = realloc(original_exit->coordinates, sizeof(Location) * original_exit->width);
        if(original_exit->coordinates == NULL)
            return FAILURE;

        original_exit->coordinates[original_exit->width - 1] = new_coordinates;

        return SUCCESS;
    }

    return FAILURE;
}

/**
 * Allocates the static_floor_field and dynamic_floor_field grids.
 * 
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
*/
Function_Status allocate_exits_set_fields()
{
    exits_set.static_floor_field = allocate_double_grid(cli_args.global_line_number, cli_args.global_column_number);
    exits_set.dynamic_floor_field = allocate_integer_grid(cli_args.global_line_number, cli_args.global_column_number);
    if(exits_set.static_floor_field == NULL || exits_set.dynamic_floor_field == NULL)
    {
        fprintf(stderr,"Failure during the allocation of the static_floor_field and dynamic_floor_field.\n");
        return FAILURE;
    }

    return SUCCESS;
}

/**
 * Calculates the static floor field as described in Annex A of Kirchner's 2002 article.
 * 
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
 */
Function_Status calculate_kirchner_static_field()
{
    int num_exit_cells = 0; // The total number of exit cells.
    Location *exit_cell_coordinates = NULL; // A list of all the exit cells coordinates.

    fill_double_grid(exits_set.static_floor_field, cli_args.global_line_number, cli_args.global_column_number, -1);
    copy_grid_structure(exits_set.static_floor_field, obstacle_grid); // Copies the structure of the environment

    // Assign a value of -1 to all exit cells. Cells marked with -1 will have their static floor field values calculated next.
    // Additionally, construct a list containing the coordinates of all exit cells to simplify further processing.
    for(int exit_index = 0; exit_index < exits_set.num_exits; exit_index++) 
    {
        Exit current_exit = exits_set.list[exit_index];
        for(int cell_index = 0; cell_index < current_exit->width; cell_index++)
        {
            Location current_cell = current_exit->coordinates[cell_index];
            exits_set.static_floor_field[current_cell.lin][current_cell.col] = -1;

            exit_cell_coordinates = realloc(exit_cell_coordinates, sizeof(Location) * (num_exit_cells + 1));
            if(exit_cell_coordinates == NULL)
            {
                fprintf(stderr, "Failure in the realloc of the exit_cells_coordinates list.\n");
                return FAILURE;
            }

            exit_cell_coordinates[num_exit_cells] = current_cell;
            num_exit_cells++;
        }
    }

    double maximum_value = -1; // The maximum euclidean distance for any cell to an exit.
    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            if(exits_set.static_floor_field[i][j] == IMPASSABLE_OBJECT)
                continue;

            for(int cell_index = 0; cell_index < num_exit_cells; cell_index++)
            {
                Location current_exit_cell = exit_cell_coordinates[cell_index]; // The current exit cell being used as the reference.
                double distance_to_exit = euclidean_distance(current_exit_cell, (Location) {i,j});

                if(exits_set.static_floor_field[i][j] == -1 || distance_to_exit < exits_set.static_floor_field[i][j])
                    exits_set.static_floor_field[i][j] = distance_to_exit;
            }

            if(exits_set.static_floor_field[i][j] > maximum_value)
                maximum_value = exits_set.static_floor_field[i][j];
        }
    }

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            if(exits_set.static_floor_field[i][j] == IMPASSABLE_OBJECT)
                continue; 

            double normalized_distance = maximum_value - exits_set.static_floor_field[i][j];
            exits_set.static_floor_field[i][j] = normalized_distance;
        }
    }

    return SUCCESS;
}

/**
 * Calculates the static weights of every exit in the exits_set.
 * 
 * @return Function_Status: FAILURE (0), SUCCESS (1) or INACCESSIBLE_EXIT(2).
*/
Function_Status calculate_all_static_weights()
{
    if(exits_set.num_exits <= 0 || exits_set.list == NULL)
    {
        fprintf(stderr,"The number of exits (%d) is invalid or the exits list is NULL.\n", exits_set.num_exits);
        return FAILURE;
    }

    for(int exit_index = 0; exit_index < exits_set.num_exits; exit_index++)
    {
        Function_Status returned_status = calculate_static_weight(exits_set.list[exit_index]);
        if(returned_status != SUCCESS )
            return returned_status;
    }

    return SUCCESS;
}

/**
 * Deallocate and reset the structures related to each exit and the exists set.
*/
void deallocate_exits()
{
    for(int exit_index = 0; exit_index < exits_set.num_exits; exit_index++)
    {
        Exit current = exits_set.list[exit_index];

        free(current->coordinates);
        deallocate_grid((void **) current->static_weight, cli_args.global_line_number);
        free(current);
    }

    free(exits_set.list);
    exits_set.list = NULL;

    deallocate_grid((void **) exits_set.static_floor_field, cli_args.global_line_number);
    deallocate_grid((void **) exits_set.dynamic_floor_field, cli_args.global_line_number);
    exits_set.static_floor_field = NULL;
    exits_set.dynamic_floor_field = NULL;

    exits_set.num_exits = 0;
}

/* ---------------- ---------------- ---------------- ---------------- ---------------- */
/* ---------------- ---------------- STATIC FUNCTIONS ---------------- ---------------- */
/* ---------------- ---------------- ---------------- ---------------- ---------------- */

/**
 * Creates a new exit structure based on the provided Location.
 * 
 * @param exit_coordinates New exit coordinates.
 * @return A NULL pointer, on error, or a Exit structure if the new exit is successfully created.
*/
static Exit create_new_exit(Location exit_coordinates)
{
    if(is_within_grid_lines(exit_coordinates.lin) && is_within_grid_columns(exit_coordinates.col))
    {
        Exit new_exit = malloc(sizeof(struct exit));
        if(new_exit != NULL)
        {
            new_exit->coordinates = malloc(sizeof(Location));
            if(new_exit->coordinates == NULL)
                return NULL;
            
            new_exit->coordinates[0] = exit_coordinates;
            new_exit->width = 1;

            new_exit->static_weight = allocate_double_grid(cli_args.global_line_number, cli_args.global_column_number);
        }

        return new_exit;
    }

    return NULL;
}

/**
 * Calculates the static weights for the given exit.
 * 
 * @param current_exit Exit for which the static weights will be calculated.
 * 
 * @return Function_Status: FAILURE (0), SUCCESS (1) or INACCESSIBLE_EXIT(2).
*/
static Function_Status calculate_static_weight(Exit current_exit)
{
    double floor_field_rule[][3] = 
            {{cli_args.diagonal,    1.0,    cli_args.diagonal},
             {       1.0,           0.0,           1.0       },
             {cli_args.diagonal,    1.0,    cli_args.diagonal}};

    initialize_static_weight_grid(current_exit);

    if(is_exit_accessible(current_exit) == false)
        return INACCESSIBLE_EXIT;

    Double_Grid static_weight = current_exit->static_weight;
    Double_Grid auxiliary_grid = allocate_double_grid(cli_args.global_line_number,cli_args.global_column_number);
    // stores the chances for the timestep t + 1

    if(auxiliary_grid == NULL)
    {
        fprintf(stderr, "Failure to allocate the auxiliary_grid at calculate_static_weight.\n");
        return FAILURE;
    }

    copy_double_grid(auxiliary_grid, static_weight); // copies the base structure of the floor field

    bool has_changed;
    do
    {
        has_changed = false;
        for(int i = 0; i < cli_args.global_line_number; i++)
        {
            for(int h = 0; h < cli_args.global_column_number; h++)
            {
                double current_cell_value = static_weight[i][h];

                if(current_cell_value == IMPASSABLE_OBJECT || current_cell_value == 0.0) // floor field calculations occur only on cells with values
                    continue;

                for(int j = -1; j < 2; j++)
                {
                    if(! is_within_grid_lines(i + j))
                        continue;
                    
                    for(int k = -1; k < 2; k++)
                    {
                        if(! is_within_grid_columns(h + k))
                            continue;

                        if(static_weight[i + j][h + k] == IMPASSABLE_OBJECT || static_weight[i + j][h + k] == EXIT_CELL)
                            continue;

                        if(j != 0 && k != 0)
                        {
                            if(! is_diagonal_valid((Location){i,h},(Location){j,k},static_weight))
                                continue;
                        }

                        double adjacent_cell_value = current_cell_value + floor_field_rule[1 + j][1 + k];
                        if(auxiliary_grid[i + j][h + k] == 0.0)
                        {    
                            auxiliary_grid[i + j][h + k] = adjacent_cell_value;
                            has_changed = true;
                        }
                        else if(adjacent_cell_value < auxiliary_grid[i + j][h + k])
                        {
                            auxiliary_grid[i + j][h + k] = adjacent_cell_value;
                            has_changed = true;
                        }
                    }
                }
            }
        }
        copy_double_grid(static_weight,auxiliary_grid); 
        // make sure static_weight now holds t + 1 timestep, allowing auxiliary_grid to hold t + 2 timestep.
    }
    while(has_changed);

    deallocate_grid((void **) auxiliary_grid, cli_args.global_line_number);

    return SUCCESS;
}

/**
 * Copies the structure (obstacles and walls) from the obstacle_grid to the static weight grid 
 * for the provided exit. Additionally, adds the exit cells to it.
 * 
 * @param current_exit The exit for which the static weights will be initialized.
*/
static void initialize_static_weight_grid(Exit current_exit)
{
    // SE current_exit for NULL, incluir todas as saídas na grid que deverá ser passada.

    // Add walls and obstacles to the static weight grid.
    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int h = 0; h < cli_args.global_column_number; h++)
        {
            double cell_value = obstacle_grid[i][h];
            if(cell_value == IMPASSABLE_OBJECT)
                current_exit->static_weight[i][h] = IMPASSABLE_OBJECT;
            else
                current_exit->static_weight[i][h] = 0.0;
        }
    }

    // Add the exit cells to the static weight grid.
    for(int i = 0; i < current_exit->width; i++)
    {
        Location exit_cell = current_exit->coordinates[i];

        current_exit->static_weight[exit_cell.lin][exit_cell.col] = EXIT_CELL;
    }
}

/**
 * Verify if the given exit is accessible.
 * 
 * @note A exit is accessible if there is, at least, one adjacent empty cell in the vertical or horizontal directions. 
 * 
 * @param current_exit The exit that will be verified.
 * @return bool, where True indicates tha the given exit is accessible, or False otherwise.
*/
static bool is_exit_accessible(Exit current_exit)
{
    if(current_exit == NULL)
        return false;

    for(int exit_cell_index = 0; exit_cell_index < current_exit->width; exit_cell_index++)
    {
        Location c = current_exit->coordinates[exit_cell_index];

        for(int j = -1; j < 2; j++)
        {
            if(! is_within_grid_lines(c.lin + j))
                continue;

            for(int k = -1; k < 2; k++)
            {
                if(! is_within_grid_columns(c.col + k))
                    continue;

                // De floor_field para static_weight AQUI
                if(current_exit->static_weight[c.lin + j][c.col + k] == IMPASSABLE_OBJECT || current_exit->static_weight[c.lin + j][c.col + k] == EXIT_CELL)
                    continue;

                if(j != 0 && k != 0)
                    continue; // diagonals

                return true;
            }
        }
    }

    return false;
}

/**
 * Identify the cells occupied by a pedestrian still in the environment and adds Cell structures with its locations and static weights to the given list.
 * 
 * @param occupied_cells A pointer to a list of Cell structures, where the Cell structures referring to occupied cells will be appended.
 * @param current_exit The exit from which the static weight values will be obtained for the occupied cells.
 * 
 * @return An integer, indicating the number of occupied cells or -1, in case of a failure.
*/
static int identify_occupied_cells(Cell **occupied_cells, Exit current_exit)
{
    int num_occupied_cells = 0;

    for(int p_index = 0; p_index < pedestrian_set.num_pedestrians; p_index++)
    {
        if(pedestrian_set.list[p_index]->state == GOT_OUT)
            continue;

        *occupied_cells = realloc(*occupied_cells, sizeof(Cell) * (num_occupied_cells + 1));
        if(*occupied_cells == NULL)
        {
            fprintf(stderr, "Failure during reallocation of the occupied_cells list in identify_occupied_cells.\n");
            return -1;
        }

        Location pedestrian_location = pedestrian_set.list[p_index]->current;

        (*occupied_cells)[num_occupied_cells].coordinates = pedestrian_location;
        (*occupied_cells)[num_occupied_cells].value = current_exit->static_weight[pedestrian_location.lin][pedestrian_location.col];

        num_occupied_cells++;
    }

    return num_occupied_cells;
}