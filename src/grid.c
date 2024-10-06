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
#include"../headers/fire_dynamics.h"
#include"../headers/cli_processing.h"
#include"../headers/shared_resources.h"

Int_Grid obstacle_grid = NULL; // Grid containing walls and obstacles.
                               // Contains cells with either IMPASSABLE_OBJECT or EMPTY_CELL values.
                               // Any cell with an exit is assigned IMPASSABLE_OBJECT value.
Int_Grid risky_cells_grid = NULL; // Grid containing either 1 for cells that are one unit distance between the corder of the fire and a wall (or impassable obstacle) or 0 otherwise. 
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
 * Assign the given value to all positions of the provided integer grid.
 *
 * @param integer_grid An integer grid to be reset. 
 * @param line_number Number of lines of the grid.
 * @param column_number Number of columns of the grid.
 * @param value The value to be assigned to all positions.
 * 
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
 */
Function_Status fill_integer_grid(Int_Grid integer_grid, int line_number, int column_number, int value)
{
    if(integer_grid == NULL)
    {
        fprintf(stderr, "The Int_Grid passed to 'fill_integer_grid' was a NULL pointer.\n");
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
            integer_grid[i][h] = value;
    }

    return SUCCESS;
}

/**
 * Assign the given value to all positions of the provided double grid.
 *
 * @param double_grid A double grid to be reset. 
 * @param line_number Number of lines of the grid.
 * @param column_number Number of columns of the grid.
 * @param value The value to be assigned to all positions.
 * 
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
 */
Function_Status fill_double_grid(Double_Grid double_grid, int line_number, int column_number, double value)
{
    if(double_grid == NULL)
    {
        fprintf(stderr, "The Double_Grid passed to 'fill_double_grid' was a NULL pointer.\n");
        return FAILURE;
    }

    for(int i = 0; i < line_number; i++)
    {
        if(double_grid[i] == NULL)
        {
            fprintf(stderr, "The line %d of the Double_Grid passed to 'fill_double_grid' was a NULL pointer.\n", i);
            return FAILURE;
        }

        for(int h = 0; h < column_number; h++)
            double_grid[i][h] = value;
    }

    return SUCCESS;
}

/**
 * Copy the content of the source grid to the destination grid.
 *
 * @param destination Integer grid where the content is to be copied.
 * @param source Integer grid to be copied.
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
 * 
 * @note Both grids must be of  global size (lines and columns). Otherwise, undefined behavior will happen.
 */
Function_Status copy_integer_grid(Int_Grid destination, Int_Grid source)
{
    if(destination == NULL || source == NULL)
    {
        fprintf(stderr, "The destination or/and source grids received by 'copy_integer_grid' was a null pointer.\n");
        return FAILURE;
    }

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        if(destination[i] == NULL || source[i] == NULL)
        {
            fprintf(stderr, "The line %d of destination or/and source in 'copy_integer_grid' was a null pointer.\n", i);
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
 * Copy the cells that aren't EMPTY_CELL from the source grid to the destination grid.
 *
 * @param destination Integer grid where the content is to be copied.
 * @param source Int grid fro which the data will be extracted.
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
 * 
 * @note Both grids must be of global size (lines and columns). Otherwise, undefined behavior will happen.
 */
Function_Status copy_non_empty_cells(Int_Grid destination, Int_Grid source)
{
    if(destination == NULL || source == NULL)
    {
        fprintf(stderr, "The destination or/and source grids received by 'copy_non_empty_cells' was a null pointer.\n");
        return FAILURE;
    }

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        if(destination[i] == NULL || source[i] == NULL)
        {
            fprintf(stderr, "The line %d of destination or/and source in 'copy_non_empty_cells' was a null pointer.\n", i);
            return FAILURE;
        }

        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            if(source[i][j] == EMPTY_CELL)
                continue;

            destination[i][j] = source[i][j];
        }
    }

    return SUCCESS;
}


/**
 * Identify the cells that aren't EMPTY_CELL from the source grid and replace the cells in the same coordinates of the destination grid with value.
 *
 * @param destination Double grid where the content is to be copied.
 * @param source Int grid fro which the data will be extracted.
 * @param value The value to be used as replacement.
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
 * 
 * @note Both grids must be of global size (lines and columns). Otherwise, undefined behavior will happen.
 */
Function_Status replace_non_empty_cells(Double_Grid destination, Int_Grid source, double value)
{
    if(destination == NULL || source == NULL)
    {
        fprintf(stderr, "The destination or/and source grids received by 'replace_non_empty_cells' was a null pointer.\n");
        return FAILURE;
    }

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        if(destination[i] == NULL || source[i] == NULL)
        {
            fprintf(stderr, "The line %d of destination or/and source in 'replace_non_empty_cells' was a null pointer.\n", i);
            return FAILURE;
        }

        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            if(source[i][j] == EMPTY_CELL)
                continue;

            destination[i][j] = value;
        }
    }

    return SUCCESS;
}

/**
 * Iterates through each cell in the source grid and adds its value to the corresponding
 * cell in the destination grid. The result is stored in place within the destination grid.
 * 
 * @param destination The grid that will be updated with the summed values.
 * @param second he grid that provides the values to be added to the destination grid.
 * 
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
 * 
 * @note Both grids must be of global size (lines and columns). Otherwise, undefined behavior will happen.
 */
Function_Status sum_grids(Int_Grid destination, Int_Grid source)
{
    if(destination == NULL || source == NULL)
    {
        fprintf(stderr, "The destination or/and source grids received by 'sum_grids' was a null pointer.\n");
        return FAILURE;
    }

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        if(destination[i] == NULL || source[i] == NULL)
        {
            fprintf(stderr, "The line %d of destination or/and source in 'sum_grids' was a null pointer.\n", i);
            return FAILURE;
        }

        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            destination[i][j] += source[i][j];
        }
    }

    return SUCCESS;
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