/* 
   File: exit.c
   Author: Daniel Gonçalves
   Date: 2023-10-15
   Description: This module contains declarations of structures to hold exit information and functions to create/expand exits, add exits to the exits set and verify if an exit is accessible.
*/

#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>

#include"../headers/exit.h"
#include"../headers/grid.h"
#include"../headers/fire_dynamics.h"
#include"../headers/pedestrian.h"
#include"../headers/cli_processing.h"
#include"../headers/static_field.h"
#include"../headers/shared_resources.h"

Location non_diagonal_modifiers[4] = {{-1, 0}, {0, -1}, {0, 1} , {1, 0}}; // The modifiers for the neighbor cells not in the diagonals.

Int_Grid exits_only_grid = NULL; // Grid containing only the exits.
                                 // Contains cells with either EXIT_CELL or EMPTY_CELL values.

Exits_Set exits_set = {NULL, NULL, NULL, NULL, 0, NULL, NULL, NULL};

static Exit create_new_exit(Location exit_coordinates);
static bool is_exit_blocked_by_fire(Exit current_exit);

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
 * Loads the structure of the environment and sets the grid cells corresponding to the current exit coordinates.
 * 
 * @note This function updates the private_structure_grid using data from the obstacle_grid and the list of exit coordinates.
 *
 * @param current_exit The exit which private_structure_grid data will be loaded.
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
 */
Function_Status set_private_grid_data(Exit current_exit)
{
    if(current_exit == NULL)
        return FAILURE;

    fill_integer_grid(current_exit->private_structure_grid, cli_args.global_line_number, cli_args.global_column_number, EMPTY_CELL);
    copy_non_empty_cells(current_exit->private_structure_grid, obstacle_grid);

    for(int cell_index = 0; cell_index < current_exit->width; cell_index++)
    {
        Location current_coordinates = current_exit->coordinates[cell_index];
        current_exit->private_structure_grid[current_coordinates.lin][current_coordinates.col] = EXIT_CELL;
    }

    return SUCCESS;
}

/**
 * Allocates the static_floor_field and dynamic_floor_field grids.
 * 
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
*/
Function_Status allocate_exits_set_fields()
{
    exits_set.static_floor_field = allocate_double_grid(cli_args.global_line_number, cli_args.global_column_number);
    exits_set.dynamic_floor_field = allocate_double_grid(cli_args.global_line_number, cli_args.global_column_number);
    exits_set.fire_floor_field = allocate_double_grid(cli_args.global_line_number, cli_args.global_column_number);
    exits_set.aux_static_grid = allocate_double_grid(cli_args.global_line_number, cli_args.global_column_number);
    exits_set.aux_dynamic_grid = allocate_double_grid(cli_args.global_line_number, cli_args.global_column_number);
    exits_set.distance_to_exits_grid = allocate_double_grid(cli_args.global_line_number, cli_args.global_column_number);
    if(exits_set.static_floor_field == NULL || exits_set.dynamic_floor_field == NULL || 
       exits_set.fire_floor_field == NULL || exits_set.aux_static_grid == NULL ||
       exits_set.aux_dynamic_grid == NULL || exits_set.distance_to_exits_grid == NULL)
    {
        fprintf(stderr,"Failure during the allocation of the exit_set double grids.\n");
        return FAILURE;
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
        deallocate_grid((void **) current->private_structure_grid, cli_args.global_line_number);
        deallocate_grid((void **) current->varas_static_weight, cli_args.global_line_number);
        free(current);
    }

    free(exits_set.list);
    exits_set.list = NULL;

    deallocate_grid((void **) exits_set.static_floor_field, cli_args.global_line_number);
    deallocate_grid((void **) exits_set.dynamic_floor_field, cli_args.global_line_number);
    deallocate_grid((void **) exits_set.fire_floor_field, cli_args.global_line_number);
    deallocate_grid((void **) exits_set.aux_static_grid, cli_args.global_line_number);
    deallocate_grid((void **) exits_set.aux_dynamic_grid, cli_args.global_line_number);
    deallocate_grid((void **) exits_set.distance_to_exits_grid, cli_args.global_line_number);
    exits_set.static_floor_field = NULL;
    exits_set.dynamic_floor_field = NULL;
    exits_set.fire_floor_field = NULL;
    exits_set.aux_static_grid = NULL;
    exits_set.aux_dynamic_grid = NULL;
    exits_set.distance_to_exits_grid = NULL;

    exits_set.num_exits = 0;
}

/**
 * Verifies if any door has been blocked by the spreading fire.
 *
 * If a door is blocked, the corresponding cells in the exit_only_grid are marked as BLOCKED_EXIT_CELL, 
 * and the flag in the respective exit structure is set to true.
 */
void check_for_exits_blocked_by_fire()
{
    for(int exit_index = 0; exit_index < exits_set.num_exits; exit_index++)
    {
        Exit current_exit = exits_set.list[exit_index];

        if(current_exit->is_blocked_by_fire)
            continue;

        if(is_exit_blocked_by_fire(current_exit))
        {
            current_exit->is_blocked_by_fire = true;

            for(int cell_index; cell_index < current_exit->width; cell_index++)
            {
                Location curr = current_exit->coordinates[cell_index];
                exits_only_grid[curr.lin][curr.col] = BLOCKED_EXIT_CELL;
            }
        }
    }
}

/**
 * Extracts the coordinates of the non-blocked exits.
 * 
 * @param num_exit_cells A pointer to a integer, where the number of exit cells in the returned array will be stored.
 * @return A pointer to an array of Locations, holding the coordinates of the non-blocked exit cells.
 */
Location *extract_non_blocked_exit_coordinates(int *num_exit_cells)
{
    Location *exit_cell_coordinates = NULL;
    *num_exit_cells = 0;

    for(int exit_index = 0; exit_index < exits_set.num_exits; exit_index++) 
    {
        Exit current_exit = exits_set.list[exit_index];

        if(current_exit->is_blocked_by_fire)
            continue;

        for(int cell_index = 0; cell_index < current_exit->width; cell_index++)
        {
            Location current_cell = current_exit->coordinates[cell_index];
            
            exit_cell_coordinates = realloc(exit_cell_coordinates, sizeof(Location) * (*num_exit_cells + 1));
            if(exit_cell_coordinates == NULL)
            {
                fprintf(stderr, "Failure in the realloc of the exit_cells_coordinates list (extract_non_blocked).\n");
                return NULL;
            }

            exit_cell_coordinates[*num_exit_cells] = current_cell;
            (*num_exit_cells)++;
        }
    }

    return exit_cell_coordinates;
}

/**
 * Computes the distance from each cell to the nearest exit cell, storing the information in the distance_to_exits_grid.
 * 
 * @param exit_cell_coordinates A list of all the valid exit cells.
 * @param num_exit_cells The number of exit cells.
 */
void calculate_distance_to_closest_exit(Location *exit_cell_coordinates, int num_exit_cells)
{
    fill_double_grid(exits_set.distance_to_exits_grid, cli_args.global_line_number, cli_args.global_column_number, -1);
    
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

                if(exits_set.distance_to_exits_grid[i][j] == -1 || distance_to_exit < exits_set.distance_to_exits_grid[i][j])
                    exits_set.distance_to_exits_grid[i][j] = distance_to_exit;
            }
        }
    }
}

/**
 * Resets, for all exits, the variable that indicates if a exit has been blocked to false.
 */
void reset_exits()
{
    for(int exit = 0; exit < exits_set.num_exits; exit++)
    {
        exits_set.list[exit]->is_blocked_by_fire = false;
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
bool is_exit_accessible(Exit current_exit)
{
    if(current_exit == NULL)
        return false;

    for(int exit_cell_index = 0; exit_cell_index < current_exit->width; exit_cell_index++)
    {
        Location c = current_exit->coordinates[exit_cell_index];

        for(int i = 0; i < 4; i++)
        {
            Location current_modifier = non_diagonal_modifiers[i];

            if(! is_within_grid_lines(c.lin + current_modifier.lin))
                continue;

            if(! is_within_grid_columns(c.col + current_modifier.col))
                continue;

            if(current_exit->private_structure_grid[c.lin + current_modifier.lin][c.col + current_modifier.col] == IMPASSABLE_OBJECT ||
               current_exit->private_structure_grid[c.lin + current_modifier.lin][c.col + current_modifier.col] == EXIT_CELL)
                continue;

            return true;
        }
    }

    return false;
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
            new_exit->is_blocked_by_fire = false;

            new_exit->varas_static_weight = allocate_double_grid(cli_args.global_line_number, cli_args.global_column_number);
            new_exit->private_structure_grid = allocate_integer_grid(cli_args.global_line_number, cli_args.global_column_number);
        }

        return new_exit;
    }

    return NULL;
}

/**
 * Verifies if the given exit has been blocked by fire. An exit is blocked by fire when all empty cell adjacent to the exit cells in the horizontal or vertical directions have been occupied by fire. 
 * 
 * @param current_exit The exit that will be verified.
 * @return bool, where True indicates tha the given exit has been blocked by fire, or False otherwise.
 */
static bool is_exit_blocked_by_fire(Exit current_exit)
{
    if(current_exit == NULL)
        return false;

    for(int exit_cell_index = 0; exit_cell_index < current_exit->width; exit_cell_index++)
    {
        Location c = current_exit->coordinates[exit_cell_index];

        for(int i = 0; i < 4; i++)
        {
            Location current_modifier = non_diagonal_modifiers[i];

            if(! is_within_grid_lines(c.lin + current_modifier.lin))
                continue;

            if(! is_within_grid_columns(c.col + current_modifier.col))
                continue;

            if(current_exit->private_structure_grid[c.lin + current_modifier.lin][c.col + current_modifier.col] == IMPASSABLE_OBJECT ||
               current_exit->private_structure_grid[c.lin + current_modifier.lin][c.col + current_modifier.col] == EXIT_CELL)
                continue;

            if(fire_grid[c.lin + current_modifier.lin][c.col + current_modifier.col] == EMPTY_CELL) // No fire in the specified cell
                return false;
        }
    }

    return true;
}