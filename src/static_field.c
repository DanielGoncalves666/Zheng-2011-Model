/* 
   File: static_field.c
   Author: Daniel Gonçalves
   Date: 15/09/2024
   Description: 
*/

#include<stdlib.h>

#include"../headers/cli_processing.h"
#include"../headers/fire_dynamics.h"
#include"../headers/static_field.h"
#include"../headers/exit.h"
#include"../headers/shared_resources.h"

static Function_Status calculate_static_weight(Exit current_exit);
static void initialize_static_weight_grid(Exit current_exit);

/**
 * Calculates the static floor field as described in Annex A of Kirchner's 2002 article.
 * 
 * @param exit_cell_coordinates A list of all the valid exit cells.
 * @param num_exit_cells The number of exit cells.
 * @param destination_grid The grid where the computed static field will be stored. If NULL is provided, the default will be exits_set.static_floor_field.
 */
void calculate_kirchner_static_field(Location *exit_cell_coordinates, int num_exit_cells, Double_Grid destination_grid)
{
    if(destination_grid == NULL)
        destination_grid = exits_set.static_floor_field;

    fill_double_grid(destination_grid, cli_args.global_line_number, cli_args.global_column_number, -1);

    double maximum_value = -1; // The maximum euclidean distance for any cell to an exit.
    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            if(exits_only_grid[i][j] == EXIT_CELL)
            {
                destination_grid[i][j] = 0; // The distance of an exit_cell to itself is 0.
                continue;
            }

            if(obstacle_grid[i][j] == IMPASSABLE_OBJECT)
            {
                destination_grid[i][j] = IMPASSABLE_OBJECT;
                continue;
            }

            for(int cell_index = 0; cell_index < num_exit_cells; cell_index++)
            {
                Location current_exit_cell = exit_cell_coordinates[cell_index]; // The current exit cell being used as the reference.
                double distance_to_exit = euclidean_distance(current_exit_cell, (Location) {i,j});

                if(destination_grid[i][j] == -1 || distance_to_exit < destination_grid[i][j])
                    destination_grid[i][j] = distance_to_exit;
            }

            if(destination_grid[i][j] > maximum_value)
                maximum_value = destination_grid[i][j];
        }
    }

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            if(destination_grid[i][j] == IMPASSABLE_OBJECT)
                continue; 

            double normalized_distance = maximum_value - destination_grid[i][j];
            destination_grid[i][j] = normalized_distance;
        }
    }
}

/**
 * Calculates the static floor field as described in the Zheng's 2011 article.
 * 
 * @param exit_cell_coordinates A list of all the valid exit cells.
 * @param num_exit_cells The number of exit cells.
 * @param destination_grid The grid where the computed static field will be stored. If NULL is provided, the default will be exits_set.static_floor_field.
 */
void calculate_zheng_static_field(Location *exit_cell_coordinates, int num_exit_cells, Double_Grid destination_grid)
{
    if(destination_grid == NULL)
        destination_grid = exits_set.static_floor_field;

    fill_double_grid(destination_grid, cli_args.global_line_number, cli_args.global_column_number, -1);

    double sum_of_all_distances = 0;
    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            if(exits_only_grid[i][j] != EXIT_CELL) // Exits cells must have their static field calculated
            {
                if(exits_only_grid[i][j] == BLOCKED_EXIT_CELL)
                {
                    destination_grid[i][j] = BLOCKED_EXIT_CELL;
                    continue;
                }

                if(obstacle_grid[i][j] == IMPASSABLE_OBJECT)
                {
                    destination_grid[i][j] = IMPASSABLE_OBJECT;
                    continue;
                }

                if(fire_grid[i][j] == FIRE_CELL)
                {
                    destination_grid[i][j] = FIRE_CELL;
                    continue;
                }
            }

            for(int cell_index = 0; cell_index < num_exit_cells; cell_index++)
            {
                Location current_exit_cell = exit_cell_coordinates[cell_index]; // The current exit cell being used as the reference.
                double distance_to_exit = euclidean_distance(current_exit_cell, (Location) {i,j});

                if(destination_grid[i][j] == -1 || distance_to_exit < destination_grid[i][j])
                    destination_grid[i][j] = distance_to_exit;
            }

            // The plus one is used to avoid division by zero when calculating the floor field for an exit cell. Some tests to verify the best number to use may be good.
            // IT'S NOT PRESENT IN THE ORIGINAL CALCULATION OF THE ZHENG FLOOR FIELD
            destination_grid[i][j] = 1 / (destination_grid[i][j] + 1);
            sum_of_all_distances += destination_grid[i][j];
        }
    }

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            if(destination_grid[i][j] == IMPASSABLE_OBJECT || destination_grid[i][j] == FIRE_CELL)
                continue;

            destination_grid[i][j] /= sum_of_all_distances;
        }
    }
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

/* ---------------- ---------------- ---------------- ---------------- ---------------- */
/* ---------------- ---------------- STATIC FUNCTIONS ---------------- ---------------- */
/* ---------------- ---------------- ---------------- ---------------- ---------------- */

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

    Double_Grid varas_static_weight = current_exit->varas_static_weight;
    Double_Grid auxiliary_grid = allocate_double_grid(cli_args.global_line_number,cli_args.global_column_number);
    // stores the chances for the timestep t + 1

    if(auxiliary_grid == NULL)
    {
        fprintf(stderr, "Failure to allocate the auxiliary_grid at calculate_static_weight.\n");
        return FAILURE;
    }

    copy_double_grid(auxiliary_grid, varas_static_weight); // copies the base structure of the floor field

    bool has_changed;
    do
    {
        has_changed = false;
        for(int i = 0; i < cli_args.global_line_number; i++)
        {
            for(int h = 0; h < cli_args.global_column_number; h++)
            {
                double current_cell_value = varas_static_weight[i][h];

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

                        if(varas_static_weight[i + j][h + k] == IMPASSABLE_OBJECT || varas_static_weight[i + j][h + k] == EXIT_CELL)
                            continue;

                        if(j != 0 && k != 0)
                        {
                            if(! is_diagonal_valid((Location){i,h},(Location){j,k},varas_static_weight))
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
        copy_double_grid(varas_static_weight,auxiliary_grid); 
        // make sure varas_static_weight now holds t + 1 timestep, allowing auxiliary_grid to hold t + 2 timestep.
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
    // Add walls and obstacles to the static weight grid.
    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int h = 0; h < cli_args.global_column_number; h++)
        {
            double cell_value = obstacle_grid[i][h];
            if(cell_value == IMPASSABLE_OBJECT)
                current_exit->varas_static_weight[i][h] = IMPASSABLE_OBJECT;
            else
                current_exit->varas_static_weight[i][h] = 0.0;
        }
    }

    // Add the exit cells to the static weight grid.
    for(int i = 0; i < current_exit->width; i++)
    {
        Location exit_cell = current_exit->coordinates[i];

        current_exit->varas_static_weight[exit_cell.lin][exit_cell.col] = EXIT_CELL;
    }
}