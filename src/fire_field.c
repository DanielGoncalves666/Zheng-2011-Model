/* 
   File: fire_Field.c
   Author: Daniel Gonçalves
   Date: 15/09/2024
   Description: 
*/

#include<stdlib.h>

#include"../headers/fire_field.h"
#include"../headers/fire_dynamics.h"
#include"../headers/grid.h"
#include"../headers/exit.h"
#include"../headers/cli_processing.h"
#include"../headers/shared_resources.h"

Double_Grid fire_distance_grid = NULL; // Grid containing the distance of every cell to the border of the fire.

typedef struct{
    int main_coordinate; // The coordinate that is common to all coordinates in the secondary_coordinates array.
    int *secondary_coordinates;
    int length; // number of secondary coordinates
}coordinate_set;

typedef struct{
    int length; // number of coordinate sets
    coordinate_set *sets;
}coordinate_set_collection;

static void calculate_distance_from_cells_to_fire();
static Function_Status add_to_coordinates_collection(coordinate_set_collection *collection, Location coordinates);
static void extract_fire_coordinate_sets(coordinate_set_collection *collection, bool line_direction);
static void deallocate_coordinate_sets(coordinate_set_collection collection);
static Function_Status determine_adjacent_coordinate_sets(coordinate_set_collection collection, int coordinate, coordinate_set **neighbor_sets, int *num_neighbors);
static Function_Status determine_adjacent_secondary_coordinates(coordinate_set *set, int coordinate, int *neighbor_coordinates, int *num_neighbors);

/**
 * Calculates the fire floor field in accordance with the 2011 Zheng's article specifications.
 */
void calculate_fire_floor_field()
{
    fill_double_grid(exits_set.fire_floor_field, cli_args.global_line_number, cli_args.global_column_number, 0);

    calculate_distance_from_cells_to_fire();

    if(! cli_args.fire_is_present)
        return; // If there is no fire, the fire floor field value is set to zero for all cells. When the pedestrian probability formula is applied, the denominator will default to 1.

    double sum_of_all_distances = 0;
    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            if(fire_distance_grid[i][j] > cli_args.fire_gamma || fire_grid[i][j] == FIRE_CELL || (obstacle_grid[i][j] != EMPTY_CELL && exits_only_grid[i][j] == EMPTY_CELL))
                continue; // Too far away from the fire, so the FF field value will be 0

            exits_set.fire_floor_field[i][j] = 1 / fire_distance_grid[i][j];
            sum_of_all_distances += exits_set.fire_floor_field[i][j];
        }
    }

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            if(exits_set.fire_floor_field[i][j] != 0)
                exits_set.fire_floor_field[i][j] /= sum_of_all_distances;
        }
    }
}

/**
 * Determine all the risky cells in the environment. A risky cell is a cell between the fire and a wall.
 */
void determine_risky_cells()
{
    fill_integer_grid(risky_cells_grid, cli_args.global_line_number, cli_args.global_column_number, NON_RISKY_CELLS);

    if(! cli_args.fire_is_present)
         return; // If there is no fire, then there is no risky or danger cells.

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            if(obstacle_grid[i][j] == IMPASSABLE_OBJECT || fire_grid[i][j] == FIRE_CELL)
               continue;
            
            if(fire_distance_grid[i][j] < 1.5) // Cells in the immediate vicinity
                risky_cells_grid[i][j] = DANGER_CELL;
        }
    }

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            // I included exits in this rule
            // I arbitrarly use 3 to ignore the walls that are too far from the fire. The value could be smaller, but with 3 I am certain all risky cells will be marked.
            if(obstacle_grid[i][j] == IMPASSABLE_OBJECT && fire_distance_grid[i][j] <= 3) 
            {
                for(int n = 0; n < 4; n++)
                {
                    int lin = i + non_diagonal_modifiers[n].lin;
                    int col = j + non_diagonal_modifiers[n].col;   

                    if(! is_within_grid_lines(lin) || ! is_within_grid_columns(col))
                        continue;

                    if(obstacle_grid[lin][col] == IMPASSABLE_OBJECT || fire_grid[lin][col] == FIRE_CELL)
                        continue;

                    // Cells with distance of 1.0 and 1.4 will be identified as risky cells.
                    if(fire_distance_grid[lin][col] < 1.5)
                        risky_cells_grid[lin][col] = RISKY_CELL;
                }
            }
        }
    }
}

/* ---------------- ---------------- ---------------- ---------------- ---------------- */
/* ---------------- ---------------- STATIC FUNCTIONS ---------------- ---------------- */
/* ---------------- ---------------- ---------------- ---------------- ---------------- */

/**
 * Calculates the distance of all the cells in the environment to the border of the fire (the distance to the closest cell with, considering the euclidean distance).
 * 
 * @note This function can be easily expanded to also register which is the closest fire cell to every cell in the environment.
 */
void calculate_distance_from_cells_to_fire()
{
    coordinate_set_collection line_set = {0, NULL}; // Collection where the main coordinate of each set is a line
    coordinate_set_collection column_set = {0, NULL}; // Collection where the main coordinate of each set is a column

    fill_double_grid(fire_distance_grid, cli_args.global_line_number, cli_args.global_column_number, 0);
    if(! cli_args.fire_is_present)
        return; // If the fire is not present, them the distance for all cells is set to 0.
    
    extract_fire_coordinate_sets(&line_set, true);
    extract_fire_coordinate_sets(&column_set, false);

    coordinate_set *neighbor_sets[3] = {NULL, NULL, NULL};
    int num_neighbor_sets = 0;
    int neighbor_coordinates[3] = {-1,-1,-1};
    int num_neighbor_coordinates = 0;

    for(int i = 0; i < cli_args.global_line_number; i++)
    {   
       for(int j = 0; j < cli_args.global_column_number; j++)
        {
            if(fire_grid[i][j] == FIRE_CELL)
                continue; // Distance remains as 0.

            double min_distance = 2e32;

            determine_adjacent_coordinate_sets(line_set, i, (coordinate_set **) neighbor_sets, &num_neighbor_sets);

            for(int line = 0; line < num_neighbor_sets; line++)
            {
                determine_adjacent_secondary_coordinates(neighbor_sets[line], j, (int *) neighbor_coordinates, &num_neighbor_coordinates);

                for(int column = 0; column < num_neighbor_coordinates; column++)
                {
                    int i_fire = neighbor_sets[line]->main_coordinate;
                    int j_fire = neighbor_coordinates[column];
                    double distance_to_fire = euclidean_distance((Location) {i,j}, (Location) {i_fire, j_fire});

                    if(distance_to_fire < min_distance)
                        min_distance = distance_to_fire;
                }
            }

            determine_adjacent_coordinate_sets(column_set, j, (coordinate_set **) neighbor_sets, &num_neighbor_sets);

            for(int column = 0; column < num_neighbor_sets; column++)
            {
                determine_adjacent_secondary_coordinates(neighbor_sets[column], i, (int *) neighbor_coordinates, &num_neighbor_coordinates);

                for(int line = 0; line < num_neighbor_coordinates; line++)
                {
                    int i_fire = neighbor_coordinates[line];
                    int j_fire = neighbor_sets[column]->main_coordinate;
                    double distance_to_fire = euclidean_distance((Location) {i,j}, (Location) {i_fire, j_fire});

                    if(distance_to_fire < min_distance)
                        min_distance = distance_to_fire;
                }
            }

            fire_distance_grid[i][j] = min_distance;
        }
    } 

    deallocate_coordinate_sets(line_set);
    deallocate_coordinate_sets(column_set);
}


/**
 * Adds the given coordinates to the specified coordinate set collection. 
 * If the main coordinate (coordinates.lin) matches the main coordinate of the last set in the collection, 
 * the secondary coordinate (coordinates.col) is appended to that set. If the main coordinate (coordinates.lin) 
 * is greater than the last set in the collection (or there is no set in the collection), a new coordinate set is created 
 * using coordinates.lin as the main coordinate and coordinates.col as the first secondary coordinate.
 * Otherwise, when the main coordinate is less than the last set, a error has occurred and the function returns FAILURE.
 * 
 * @param collection A pointer to the collection of coordinate sets to which the given coordinates will be added.
 * @param coordinates The coordinates to be added to the collection. 
 *                    coordinates.lin represents the main coordinate, and coordinates.col represents the secondary coordinate.
 * 
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
 */
static Function_Status add_to_coordinates_collection(coordinate_set_collection *collection, Location coordinates)
{
    if(collection->length == 0 || collection->sets[collection->length - 1].main_coordinate < coordinates.lin)
    {
        // The collection is empty or the main coordinate (coordinates.lin) is greater than the main coordinate of the last set in the collection.

        collection->length++;
        collection->sets = realloc(collection->sets, sizeof(coordinate_set) * collection->length);

        coordinate_set *current_set = &(collection->sets[collection->length - 1]);
        current_set->main_coordinate = coordinates.lin;
        current_set->length = 1;
        current_set->secondary_coordinates = malloc(sizeof(int));
        current_set->secondary_coordinates[0] = coordinates.col;

        return SUCCESS;
    }

    if(collection->sets[collection->length - 1].main_coordinate < coordinates.lin)
        return FAILURE; // The coordinates.lin value must be equal or greater than the last main_coordinate of the collection

    // The last main_coordinate is equal to coordinates.lin so the correspondent secondary_coordinates array is expanded.

    coordinate_set *current_set = &(collection->sets[collection->length - 1]);
    current_set->length += 1;
    current_set->secondary_coordinates = realloc( current_set->secondary_coordinates, sizeof(int) * current_set->length);
    current_set->secondary_coordinates[current_set->length - 1] = coordinates.col;
        
    return SUCCESS;
}

/**
 * Populates a coordinate_set_collection with the positions of all fire occurrences in the fire grid, 
 * scanning in the specified direction.
 * 
 * @param collection A pointer to the coordinate_set_collection structure where all information extracted will be stored.
 * @param line_direction Indicates if the extraction should follow the line_direction (true), where a line is processed from beginning to end, or column_direction (false), where a column is processed from beginning to end.
 */
static void extract_fire_coordinate_sets(coordinate_set_collection *collection, bool line_direction)
{
    int first_dimension_limit = line_direction ? cli_args.global_line_number : cli_args.global_column_number;
    int second_dimension_limit = line_direction ? cli_args.global_column_number : cli_args.global_line_number;
    // First dimension is the direction the scanning is undergoing.

    for(int i = 0; i < first_dimension_limit; i++)
    {
        for(int j = 0; j < second_dimension_limit; j++)
        {
            if(fire_grid[i][j] == EMPTY_CELL)
                continue;

            add_to_coordinates_collection(collection, (line_direction ? (Location) {i,j} : (Location) {j, i}));         
        }
    }
}


/**
 * Deallocate all memory used for the given coordinate_set_collection sets.
 * 
 * @param collection The set collection whose allocated memory will be deallocated.
 */
static void deallocate_coordinate_sets(coordinate_set_collection collection)
{
    for(int i = 0; i < collection.length; i++)
        free(collection.sets[i].secondary_coordinates);

    free(collection.sets);
}

/**
 * Searches the given collection for coordinate sets whose main coordinates are closest to the specified `coordinate`, 
 * using a binary search approach.
 * 
 * 1 - If `coordinate` is less than the main coordinate of the first set, the only neighbor set is the first set.
 * 2 - If `coordinate` is greater than the main coordinate of the last set, the only neighbor set is the last set.
 * 3 - If `coordinate` is between the main coordinates of two sets, both sets are considered its neighbors.
 * 4 - If `coordinate` is equal to the main coordinate of a set, the neighbors include the corresponding set, the previous set, 
 *   and the next set (if they exist).
 * 
 * @param collection The collection of coordinate sets where the search will be performed.
 * @param coordinate The coordinate whose "neighbors" are being searched for.
 * @param neighbor_sets A pointer to an array of 3 pointers where the addresses of the up to three closest sets will be stored.
 * @param num_neighbors A pointer to an integer, where the number of neighbors identified will be stored.
 * 
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
 */
static Function_Status determine_adjacent_coordinate_sets(coordinate_set_collection collection, int coordinate, coordinate_set **neighbor_sets, int *num_neighbors)
{
    if(neighbor_sets == NULL)
        return FAILURE;

    int left = 0;
    int right = collection.length - 1;
    int middle = -1;

    while(left <= right)
    {
        middle = (left + right) / 2;

        int current_main_coordinate = collection.sets[middle].main_coordinate;
        if(current_main_coordinate == coordinate) // Case 4
        {
            *num_neighbors = 0;

            if(middle - 1 >= 0)
                neighbor_sets[(*num_neighbors)++] = &(collection.sets[middle - 1]);

            neighbor_sets[(*num_neighbors)++] = &(collection.sets[middle]);

            if(middle + 1 <= collection.length - 1)
                neighbor_sets[(*num_neighbors)++] = &(collection.sets[middle + 1]);

            return SUCCESS;
        }

        if(current_main_coordinate < coordinate)
            left = middle + 1;
        else if(current_main_coordinate > coordinate)
            right = middle - 1;
    }

    if(left == 0) // Case 1
    {
        neighbor_sets[0] = &(collection.sets[0]);
        *num_neighbors = 1;
    }else if(left == collection.length) // Case 2
    {
        neighbor_sets[0] = &(collection.sets[collection.length - 1]);
        *num_neighbors = 1;
    }else if(right == left - 1) // Case 3
    {
        neighbor_sets[0] = &(collection.sets[right]);
        neighbor_sets[1] = &(collection.sets[left]);
        *num_neighbors = 2;
    }

    return SUCCESS;
}

/**
 * Searches the given coordinate set for the secondary coordinates closest to the specified `coordinate`, 
 * using a binary search approach.
 * 
 * 1 - If `coordinate` is less than the first secondary coordinate, the only neighbor is the first coordinate.
 * 2 - If `coordinate` is greater than the last secondary coordinate, the only neighbor is the last coordinate.
 * 3 - If `coordinate` is between two secondary coordinates, both coordinates are considered its neighbors.
 * 4 - If `coordinate` is equal to a secondary coordinate, the neighbors include the corresponding coordinate, the previous coordinate, 
 *   and the next coordinate (if they exist).
 * 
 * @param set A pointer to the coordinate set where the search will be performed.
 * @param coordinate The coordinate whose "neighbors" are being searched for.
 * @param neighbor_coordinates A pointer to an array of 3 positions where the (up to three) secondary coordinates will be stored.
 * @param num_neighbors A pointer to an integer, where the number of neighbors identified will be stored.
 */
static Function_Status determine_adjacent_secondary_coordinates(coordinate_set *set, int coordinate, int *neighbor_coordinates, int *num_neighbors)
{
    if(neighbor_coordinates == NULL)
        return FAILURE;

    int left = 0;
    int right = set->length - 1;
    int middle = -1;

    while(left <= right)
    {
        middle = (left + right) / 2;

        int current_secondary_coordinate = set->secondary_coordinates[middle];
        if(current_secondary_coordinate == coordinate) // Case 4
        {
            *num_neighbors = 0;

            if(middle - 1 >= 0)
                neighbor_coordinates[(*num_neighbors)++] = set->secondary_coordinates[middle - 1];

            neighbor_coordinates[(*num_neighbors)++] = current_secondary_coordinate;

            if(middle + 1 <= set->length - 1)
                neighbor_coordinates[(*num_neighbors)++] = set->secondary_coordinates[middle + 1];;

            return SUCCESS;
        }

        if(current_secondary_coordinate < coordinate)
            left = middle + 1;
        else if(current_secondary_coordinate > coordinate)
            right = middle - 1;
    }

    if(left == 0) // Case 1
    {
        neighbor_coordinates[0] = set->secondary_coordinates[0];
        *num_neighbors = 1;
    }else if(left == set->length) // Case 2
    {
        neighbor_coordinates[0] = set->secondary_coordinates[set->length - 1];
        *num_neighbors = 1;
    }else if(right == left - 1) // Case 3
    {
        neighbor_coordinates[0] = set->secondary_coordinates[right];
        neighbor_coordinates[1] = set->secondary_coordinates[left];
        *num_neighbors = 2;
    }

    return SUCCESS;
}
