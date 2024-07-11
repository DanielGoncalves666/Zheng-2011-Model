/* 
   File: grid.c
   Author: Daniel Gonçalves
   Date: 2024-05-20
   Description: This module contains the declaration of grid types for integer and floating-point numbers, as well as functions to allocate, reset, copy, test limits, verify diagonal validity and deallocate those grids.
*/

#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>

#include"../headers/grid.h"
#include"../headers/cli_processing.h"
#include"../headers/shared_resources.h"

Int_Grid environment_only_grid = NULL; // Grid containing only the structure and exits.
Int_Grid pedestrian_position_grid = NULL; // Grid containing pedestrians at their respective positions.
Int_Grid heatmap_grid = NULL; // Grid containing the count of pedestrian visits per cell.

/**
 * Dynamically allocates an integer matrix of dimensions determined by the function parameters.
 * 
 * @param line_number Number of lines of the grid.
 * @param column_number Number of columns of the grid.
 * @return A NULL pointer, on error, or an Integer_Grid if the grid was successfully allocated.
 * 
 * @note All positions of the matrix are already zeroed.
 */
Int_Grid allocate_integer_grid(int line_number, int column_number)
{
    if(line_number <= 0 || column_number <= 0)
    {
        fprintf(stderr, "At least one of the grid dimensions was negative or zero.\n");
        return NULL;
    }

    Int_Grid new_grid = malloc(sizeof(int *) * line_number);
    if( new_grid == NULL )
    {
        fprintf(stderr, "Failed to allocate memory for the lines of an integer grid.\n");
        return NULL;
    }
    
    for(int i = 0; i < line_number; i++)
    {
        new_grid[i] = calloc(column_number, sizeof(int));
        if(new_grid[i] == NULL)
        {
            deallocate_grid((void **) new_grid, i);

            fprintf(stderr, "Failed to allocate memory for the columns of the line %d of an integer grid.\n", i);
            return NULL;
        }
    }

    return new_grid;
}

/**
 * Dynamically allocates a double matrix of dimensions determined by the function parameters.
 *
 * @param line_number Number of lines of the grid.
 * @param column_number Number of columns of the grid.
 * @return A NULL pointer, on error, or an Double_Grid if the grid was successfully allocated.
 * 
 * @note All positions of the matrix are already zeroed.
 */
Double_Grid allocate_double_grid(int line_number, int column_number)
{
    if(line_number <= 0 || column_number <= 0)
    {
        fprintf(stderr, "At least one of the grid dimensions was negative or zero.\n");
        return NULL;
    }

    double **new_grid = malloc(sizeof(double *) * line_number);
    if( new_grid == NULL )
    {
        fprintf(stderr, "Failed to allocate memory for the lines of a double grid.\n");
        return NULL;
    }
    
    for(int i = 0; i < line_number; i++)
    {
        new_grid[i] = calloc(column_number, sizeof(double));
        if(new_grid[i] == NULL)
        {
            deallocate_grid((void **) new_grid, i);

            fprintf(stderr, "Failed to allocate memory for the columns of the line %d of a double grid.\n", i);
            return NULL;
        }
    }

    return new_grid;
}

/**
 * Reset all positions of an integer grid to zero.
 *
 * @param integer_grid An integer grid to be reset. 
 * @param line_number Number of lines of the grid.
 * @param column_number Number of columns of the grid.
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
 */
Function_Status reset_integer_grid(Int_Grid integer_grid, int line_number, int column_number)
{
    if(integer_grid == NULL)
    {
        fprintf(stderr, "The Int_Grid passed to 'reset_integer_grid' was a NULL pointer.\n");
        return FAILURE;
    }

    for(int i = 0; i < line_number; i++)
    {
        if(integer_grid[i] == NULL)
        {
            fprintf(stderr, "The line %d of the Int_Grid passed to 'reset_integer_grid was a NULL pointer.\n", i);
            return FAILURE;
        }

        for(int h = 0; h < column_number; h++)
            integer_grid[i][h] = 0;
    }

    return SUCCESS;
}

/**
 * Reset all positions of a double grid to zero.
 *
 * @param double_grid A double grid to be reset. 
 * @param line_number Number of lines of the grid.
 * @param column_number Number of columns of the grid.
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
 */
Function_Status reset_double_grid(Double_Grid double_grid, int line_number, int column_number)
{
    if(double_grid == NULL)
    {
        fprintf(stderr, "The Double_Grid passed to 'reset_double_grid' was a NULL pointer.\n");
        return FAILURE;
    }

    for(int i = 0; i < line_number; i++)
    {
        if(double_grid[i] == NULL)
        {
            fprintf(stderr, "The line %d of the Double_Grid passed to 'reset_double_grid' was a NULL pointer.\n", i);
            return FAILURE;
        }

        for(int h = 0; h < column_number; h++)
            double_grid[i][h] = 0.0;
    }

    return SUCCESS;
}

/**
 * Copy the content of the source grid to the destination grid.
 *
 * @param destination Double grid where the content is to be copied.
 * @param source Double grid to be copied.
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
 * 
 * @note Both grids must be of  global size (lines and columns). Otherwise, undefined behavior will happen.
 */
Function_Status copy_double_grid(Double_Grid destination, Double_Grid source)
{
    if(destination == NULL || source == NULL)
    {
        fprintf(stderr, "The destination or/and source grids received by 'copy_double_grid' was a null pointer.\n");
        return FAILURE;
    }

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        if(destination[i] == NULL || source[i] == NULL)
        {
            fprintf(stderr, "The line %d of destination or/and source in 'copy_double_grid' was a null pointer.\n", i);
            return FAILURE;
        }

        for(int h = 0; h < cli_args.global_column_number; h++)
        {
            destination[i][h] = source[i][h];
        }
    }

    return SUCCESS;
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
    floor_field[origin_cell.lin + coordinate_modifier.lin][origin_cell.col] == WALL_VALUE)
    {
        is_vertical_blocked = true;
    }

    if(is_within_grid_columns(origin_cell.col + coordinate_modifier.col) && 
    floor_field[origin_cell.lin][origin_cell.col + coordinate_modifier.col] == WALL_VALUE)
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
 * Deallocate all memory assigned to a integer grid.
 *
 * @param grid An integer or double grid, casted to (void **).
 * @param line_number Number of lines of the grid that should be deallocated.
 */
void deallocate_grid(void **grid, int line_number)
{
    if(grid != NULL)
    {
        for(int i = 0; i < line_number; i++)
        {
            free(grid[i]);
        }
        free(grid);

        grid = NULL;
    }
}