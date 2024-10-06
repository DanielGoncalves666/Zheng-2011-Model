/* 
   File: global_declarations.c
   Author: Daniel Gonçalves
   Date: 2023-10-15
   Description: This module contains declarations of enums, structures, constants, and functions that are used throughout the program.
*/

#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>
#include<math.h>

#include"../headers/grid.h"
#include"../headers/cli_processing.h"
#include"../headers/shared_resources.h"

#include"../headers/fire_dynamics.h"
#include"../headers/pedestrian.h"
#include"../headers/exit.h"

/**
 * Verifies if the environment_origin selected uses data extracted from an auxiliary file.
 * 
 * @return bool, where True indicates that auxiliary data is used and False otherwise.
*/
bool origin_uses_auxiliary_data()
{
    return cli_args.environment_origin == ONLY_STRUCTURE || 
           cli_args.environment_origin == STRUCTURE_AND_PEDESTRIANS || 
           cli_args.environment_origin == AUTOMATIC_CREATED;
}

/**
 * Verifies if the environment_origin selected uses pedestrians loaded directly from the env-file instead of randomly inserting them.
 * 
 * @return bool, where True indicates that the origin uses static pedestrians and False otherwise.
*/
bool origin_uses_static_pedestrians()
{
    return cli_args.environment_origin == STRUCTURE_AND_PEDESTRIANS || 
           cli_args.environment_origin == STRUCTURE_DOORS_AND_PEDESTRIANS;
}

/**
 * Verifies if the selected environment_origin uses exits loaded directly from the env-file instead of inserting them with data from an auxiliary file.
 * 
 * @return bool, where True indicates that the origin uses static exits and False otherwise.
*/
bool origin_uses_static_exits()
{
    return cli_args.environment_origin == STRUCTURE_AND_DOORS || 
           cli_args.environment_origin == STRUCTURE_DOORS_AND_PEDESTRIANS;
}

/**
 * Verifies if the given Locations contain the same coordinates
 * 
 * @param first A Location.
 * @param second A Location.
 * 
 * @return True, if the coordinates are the name, or False, if otherwise.
 */
bool are_same_coordinates(Location first, Location second)
{
    return first.lin == second.lin && first.col == second.col;
}

/**
 * Calculates the Euclidean distance between the provided coordinates.
 * @param first The first pair of coordinates.
 * @param second The second pair of coordinates.
 * 
 * @return A double, the euclidean distance between the two pair of coordinates provided.
 */
double euclidean_distance(Location first, Location second)
{
    return sqrt(pow(first.lin - second.lin, 2) + pow(first.col - second.col, 2));
}

/**
 * Generates a random floating-point number within a specified range [min, max].
 * 
 * @param min The lower bound of the range (inclusive).
 * @param max The upper bound of the range (inclusive).
 * 
 * @return A random floating-point number between `min` and `max`, inclusive.
 */
float rand_within_limits(float min, float max) 
{
    float range = (max - min); 
    double div = RAND_MAX / range;
    return min + (rand() / div);
}

/**
 * Determines whether a event occurs based on a probability.
 * 
 * @param probability The probability that an event will occur. Must be a value between 0 and 1 (inclusive).
 * 
 * @return True, if the draw number is less than the given probability (i.e., the event happens), or False, if otherwise.
 */
bool probability_test(double probability)
{
    float draw_number = rand_within_limits(0,1);

    return draw_number < probability;
}

/**
 * Simulates a roulette wheel selection to determine which of the provided probabilities (each corresponding to a specific event) will be chosen. 
 * 
 * @param probability_list A list of probabilities, from which one will be chosen.
 * @param length The number of elements in the probability_list
 * @param total_probability The sum of all the probabilities in the probability_list.
 * 
 * @return The index of the "chosen probability".
 */
int roulette_wheel_selection(double *probability_list, int length, double total_probability)
{
    float draw_value = 0;
    int index = -1; // The index of the chosen "probability".

    draw_value = rand_within_limits(0,total_probability);

    double current_sum = 0; // Sum of probabilities from index 0 to i.
    for(int i = 0; i < length; i++)
    { 
        if(probability_list[i] == 0)
            continue;

        index = i; // Stores the last valid index. Used to return the last valid probability in case of rounding errors.

        current_sum += probability_list[i];

        if(draw_value <= current_sum + TOLERANCE)
            return index;
    }

    return index; // Will reach in case of rounding errors.
}

/**
 * Verifies if a diagonal beginning at origin_cell and ending at origin_cell + coordinate_modifier is valid for crossing 
 * in the given floor field. 
 * If there are obstacles on both sides, then the diagonal is not valid. If the prevent_corner_crossing flag is True, 
 * then diagonals with at least one obstacle on its sides are not valid.
 *
 * @param origin_cell Origin cell coordinates. Represents where a pedestrian is or a cell whose neighborhood is being calculated.
 * @param coordinate_modifier Line and column coordinate modifiers. They are added to the origin cell coordinates, and the final 
 * result represents one of the four diagonal cells in the origin cell's neighborhood.
 * @param floor_field A Double_Grid representing a floor field.
 * @return bool, where True indicates that a diagonal is valid and False otherwise.
 */
bool is_diagonal_valid(Location origin_cell, Location coordinate_modifier, Double_Grid floor_field)
{
    bool is_horizontal_blocked = false; // Indicates if the horizontal cell in the origin_cell's neighborhood, which is adjacent to origin_cell + coordinate_modifier, is blocked.
    bool is_vertical_blocked = false;// Indicates if the vertical cell in the origin_cell's neighborhood, which is adjacent to origin_cell + coordinate_modifier, is blocked.

    if(is_within_grid_lines(origin_cell.lin + coordinate_modifier.lin) && 
    floor_field[origin_cell.lin + coordinate_modifier.lin][origin_cell.col] == IMPASSABLE_OBJECT)
    {
        is_vertical_blocked = true;
    }

    if(is_within_grid_columns(origin_cell.col + coordinate_modifier.col) && 
    floor_field[origin_cell.lin][origin_cell.col + coordinate_modifier.col] == IMPASSABLE_OBJECT)
    {
        is_horizontal_blocked = true;
    }

    if(is_vertical_blocked && is_horizontal_blocked)
        return false; // The diagonal cell is completely blocked.

    if(cli_args.prevent_corner_crossing && (is_vertical_blocked || is_horizontal_blocked))
        return false; // The diagonal is blocked by the corner of one obstacle. The prevent_corner_crossing flag indicates that this condition validates as a blocked diagonal or not.

    return true;
}

/**
 * Verifies if the value passed to the function is within the grid lines limits, i. e., 0 <= line_coordinate < cli_args.global_line_number.
 * 
 * @param line_coordinate Line coordinate to be tested.
 * @return bool, where True indicates that the value passed is within limits, or False otherwise.
*/
bool is_within_grid_lines(int line_coordinate)
{
    return line_coordinate >= 0 && line_coordinate < cli_args.global_line_number;
}

/**
 * Verifies if the value passed to the function is within the grid column limits, i. e., 
 * 0 <= column_coordinate < cli_args.global_column_number.
 * 
 * @param column_coordinate Column coordinate to be tested.
 * @return bool, where True indicates that the value passed is within limits, or False otherwise.
*/
bool is_within_grid_columns(int column_coordinate)
{
    return column_coordinate >= 0 && column_coordinate < cli_args.global_column_number;
}


/**
 * Verifies if the cell in the given location is empty (i.e., not occupied by a pedestrian, door, obstacle, wall or fire).
 * 
 * @param coordinates The coordinates of the cell
 * @return bool, where True indicates that the cell is indeed empty, or False otherwise.
 */
bool is_cell_empty(Location coordinates)
{
    if(pedestrian_position_grid[coordinates.lin][coordinates.col] != 0)
        return false;

    if(obstacle_grid[coordinates.lin][coordinates.col] != EMPTY_CELL)
        return false;

    if(exits_only_grid[coordinates.lin][coordinates.col] != EMPTY_CELL)
        return false;

    if(fire_grid[coordinates.lin][coordinates.col] != EMPTY_CELL)
        return false;

    return true;
}

/**
 * Verifies if the cell in the given location has a fire.
 * 
 * @param coordinates The coordinates of the cell
 * @return bool, where True indicates that the cell has a fire, or False otherwise.
 */
bool is_cell_with_fire(Location coordinates)
{
    return fire_grid[coordinates.lin][coordinates.col] == FIRE_CELL;
}