/* 
   File: pedestrian.c
   Author: Daniel Gonçalves
   Date: 2023-10-15
   Description: This module contains struct declarations related to pedestrians, as well as functions to create and deallocate pedestrian structures, determine which pedestrians are in panic, calculate their movements, identify and resolve conflicts, and reset parts or all of the pedestrian structure if necessary.
*/

#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>
#include<math.h>

#include"../headers/cell.h"
#include"../headers/exit.h"
#include"../headers/grid.h"
#include"../headers/pedestrian.h"
#include"../headers/cli_processing.h"
#include"../headers/shared_resources.h"

#define PANIC_PROBABILITY 0.05

typedef struct reduced_line_equation{
    double angular_coefficient;
    double linear_coefficient; // Where the line intersects the y-axis.
}reduced_line_equation;

typedef struct cell_conflict{
    int num_pedestrians;
    int pedestrian_ids[8];
    int pedestrian_allowed;
}cell_conflict;

Pedestrian_Set pedestrian_set = {NULL,0};

static Pedestrian create_pedestrian(Location ped_coordinates);
static bool are_pedestrian_paths_crossing(Pedestrian first_pedestrian, Pedestrian second_pedestrian);
static Function_Status calculate_reduced_line_equation(Location origin, Location target, reduced_line_equation* line);
static void calculate_intersection_point(reduced_line_equation first_line, reduced_line_equation second_line, double *x, double *y);
static bool is_intersection_within_pedestrian_movement(double x_coordinate, double y_coordinate, Pedestrian pedestrian);
static void solve_X_movement(Pedestrian first_pedestrian, Pedestrian second_pedestrian);

/**
 * Inserts a specified number of pedestrians at random locations within the environment.
 * 
 * @note This function does not handle cases where there is insufficient space to insert all pedestrians.
 * 
 * @param num_pedestrians_to_insert Number of pedestrians to insert in the environment.
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
*/
Function_Status insert_pedestrians_at_random(int num_pedestrians_to_insert)
{
    if(num_pedestrians_to_insert <= 0)
    {
        fprintf(stderr, "The number os pedestrians to randomly insert in the environment must be greater than 0.\n");
        return FAILURE;
    }

    if(initialize_integer_grid(pedestrian_position_grid, cli_args.global_line_number, cli_args.global_column_number, 0) == FAILURE)
        return FAILURE;

    for(int p_index = 0; p_index < num_pedestrians_to_insert;)
    {
        int line = rand() % (cli_args.global_line_number - 1) + 1;
        int column = rand() % (cli_args.global_column_number - 1) + 1;

        Location random_coordinates = {line,column};

        if(pedestrian_position_grid[line][column] != 0 || exits_set.final_floor_field[line][column] == EXIT_VALUE 
            || exits_set.final_floor_field[line][column] == WALL_VALUE)
            continue;

        if( add_new_pedestrian(random_coordinates) == FAILURE)
            return FAILURE;

        pedestrian_position_grid[line][column] = pedestrian_set.list[pedestrian_set.num_pedestrians - 1]->id;

        p_index++;
    }

    return SUCCESS;
}

/**
 * Adds a new pedestrian to the pedestrian set.
 * 
 * @note The ID of the newly created pedestrian is given in this function.
 * 
 * @param ped_coordinates New pedestrian coordinates.
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
*/
Function_Status add_new_pedestrian(Location ped_coordinates)
{
    Pedestrian new_pedestrian = create_pedestrian(ped_coordinates);
    if(new_pedestrian == NULL)
    {
        fprintf(stderr, "Failure on creating a pedestrian at coordinates (%d,%d).\n", ped_coordinates.lin, ped_coordinates.col);
        return FAILURE;
    }

    pedestrian_set.num_pedestrians += 1;
    pedestrian_set.list = realloc(pedestrian_set.list, sizeof(struct pedestrian) * pedestrian_set.num_pedestrians);
    if(pedestrian_set.list == NULL)
    {
        fprintf(stderr,"Failure in the realloc of the pedestrian_set list.\n");
        return FAILURE;
    }

    new_pedestrian->id = pedestrian_set.num_pedestrians;
    pedestrian_set.list[pedestrian_set.num_pedestrians - 1] = new_pedestrian;

    return SUCCESS;
}


/**
 * Deallocate the pedestrian_set list and reset the number of pedestrians.
*/
void deallocate_pedestrians()
{
    for(int p_index = 0; p_index < pedestrian_set.num_pedestrians; p_index++)
        free(pedestrian_set.list[p_index]);
        
    free(pedestrian_set.list);
    pedestrian_set.list = NULL;

    pedestrian_set.num_pedestrians = 0;
}

/**
 * For each pedestrian, determines if they will enter a panic state with a probability defined by PANIC_PROBABILITY.
 * If a pedestrian enters panic, they will remain in the same position during the current timestep.
 * 
 * @return A integer, indicating the number of pedestrians in panic.
*/
int determine_pedestrians_in_panic()
{
    int num_pedestrians_in_panic = 0;
    for(int p_index = 0; p_index < pedestrian_set.num_pedestrians; p_index++)
    {
        if(pedestrian_set.list[p_index]->state == GOT_OUT)
            continue;

        if((rand() % 100 + 1) / 100.0 <= PANIC_PROBABILITY)
        {
            pedestrian_set.list[p_index]->in_panic = true;
            num_pedestrians_in_panic++;

            if(cli_args.show_debug_information)
                printf("%d in panic.\n", pedestrian_set.list[p_index]->id);
        }
    }

    return num_pedestrians_in_panic;
}

/**
 * Determines the destination cell for each pedestrian.
*/
void evaluate_pedestrians_movements()
{
    for(int p_index = 0; p_index < pedestrian_set.num_pedestrians; p_index++)
    {
        Pedestrian current_pedestrian = pedestrian_set.list[p_index];

        if(current_pedestrian->state != MOVING || current_pedestrian->in_panic == true)
            continue;

        Cell destination_cell = find_smallest_cell(current_pedestrian->current, true);

        if(destination_cell.coordinates.lin == -1 && destination_cell.coordinates.col == -1)
        { 
            // There isn't a valid cell to move.
            current_pedestrian->state = STOPPED;
        
            if(cli_args.show_debug_information)
                printf("%d has been cornered.\n", current_pedestrian->id);
        }
        else
        {
            current_pedestrian->target.lin = destination_cell.coordinates.lin;
            current_pedestrian->target.col = destination_cell.coordinates.col;
        }
    }
}

/**
 * Verifies the target cells of all pedestrians and identifies cases where multiple pedestrians aim to move to the same cell.
 * 
 * @param pedestrian_conflicts A pointer to a pointer to a cell_conflict structure, representing the address of a list of cell_conflict structures. The function will create this list of conflicts and assign its pointer to the provided pointer. 
 * @param num_conflicts Pointer to a integer, where the number of conflicts will be stored.
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
*/
Function_Status identify_pedestrian_conflicts(Cell_Conflict *pedestrian_conflicts, int *num_conflicts)
{
    int conflict_number = 0;
    Int_Grid conflict_grid = allocate_integer_grid(cli_args.global_line_number,cli_args.global_column_number);
    if(conflict_grid == NULL)
    {
        fprintf(stderr, "Failure in the allocation of the conflict_grid.\n");
        return FAILURE;
    }

    Cell_Conflict conflict_list = NULL;

    for(int p_index = 0; p_index < pedestrian_set.num_pedestrians; p_index++)
    {
        Pedestrian current_pedestrian = pedestrian_set.list[p_index];

        if(current_pedestrian->state != MOVING  || current_pedestrian->in_panic == true)
            continue;

        int *target_cell = &(conflict_grid[current_pedestrian->target.lin][current_pedestrian->target.col]); 

        if(*target_cell == 0) // No previous pedestrian has the same target cell.
        {
            // The pedestrian's ID is written into the target cell to indicate his intention to move there.
            *target_cell = current_pedestrian->id;
            continue;
        }

        if(*target_cell > 0) // Exactly one pedestrian has the same target cell (so far).
        {
            // A new conflict has been found. A cell_conflict structure is created and filled.
            conflict_list = realloc(conflict_list, sizeof(cell_conflict) * (conflict_number + 1));
            if(conflict_list == NULL)
            {
                fprintf(stderr,"Failure in the realloc of the conflict_list.\n");
                return FAILURE;
            }

            Cell_Conflict current_conflict = &(conflict_list[conflict_number]);

            current_conflict->pedestrian_ids[0] = *target_cell;
            current_conflict->pedestrian_ids[1] = current_pedestrian->id;
            current_conflict->num_pedestrians = 2;

            conflict_number++;

            conflict_grid[current_pedestrian->target.lin][current_pedestrian->target.col] = conflict_number * -1;
            // conflict_number - 1 indicates the index of the current conflict in the conflict_list.
            // To recover the newly created cell_conflict structure if another pedestrian targets the same cell,
            // a negative number is written in the conflict_grid. This number can be used to extract the index..

            continue;
        }

        // The value of *target_cell is less than 0. This indicates that a conflict for the target_cell already exists. 
        // Futhermore, the corresponding index of the cell_conflict for this cell can be obtained by the following expression.

        int conflict_index = (*target_cell * -1) - 1;
        Cell_Conflict current_conflict = &(conflict_list[conflict_index]);

        current_conflict->pedestrian_ids[current_conflict->num_pedestrians] = current_pedestrian->id;
        current_conflict->num_pedestrians++;
        // Adds the new id to the cell_conflict structure.
    }

    deallocate_grid((void **) conflict_grid,cli_args.global_line_number);

    *pedestrian_conflicts = conflict_list;
    *num_conflicts = conflict_number;

    return SUCCESS;
}

/**
 * For each of the conflicts in the provided cell_conflict list decides which of the pedestrians will be allowed to move to the targeted cell. 
 * 
 * @param pedestrian_conflicts A pointer to a cell_conflict structure, representing a list of cell_conflict structures. 
 * @param num_conflicts The number of cell_conflict structures in pedestrian_conflicts list.
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
*/
Function_Status solve_pedestrian_conflicts(Cell_Conflict pedestrian_conflicts, int num_conflicts)
{
    if(pedestrian_conflicts == NULL && num_conflicts > 0)
    {
        fprintf(stderr, "Null pointer received at solve_pedestrian_conflicts when a valid pointer was expected.\n");
        return FAILURE;
    }

    for(int conflict_index = 0; conflict_index < num_conflicts; conflict_index++)
    {
        Cell_Conflict current_conflict = &(pedestrian_conflicts[conflict_index]);
        int random_result = rand() % current_conflict->num_pedestrians;

        current_conflict->pedestrian_allowed = current_conflict->pedestrian_ids[random_result];
        for(int p_index = 0; p_index < current_conflict->num_pedestrians; p_index++)
        {
            int pedestrian_id = current_conflict->pedestrian_ids[p_index] - 1;

            if(random_result != p_index)
                pedestrian_set.list[pedestrian_id]->state = STOPPED;
        }
    }

    return SUCCESS;
}

/**
 * Prints the pedestrians involved in each conflict and the pedestrian who was allowed to move.
 * 
 * @param pedestrian_conflicts A pointer to a cell_conflict structure, representing a list of cell_conflict structures. 
 * @param num_conflicts The number of cell_conflict structures in pedestrian_conflicts list.
*/
void print_pedestrian_conflict_information(Cell_Conflict pedestrian_conflicts, int num_conflicts)
{
	if(pedestrian_conflicts != NULL)
	{
		for(int conflict_index = 0; conflict_index < num_conflicts; conflict_index++)
		{
            cell_conflict current_conflict = pedestrian_conflicts[conflict_index];

			printf("Conflict %d: ", conflict_index);
			for(int p_index = 0; p_index < current_conflict.num_pedestrians; p_index++)
            {
                printf("%d ", current_conflict.pedestrian_ids[p_index]);
            }
            printf("--> %d\n", current_conflict.pedestrian_allowed);
		}
	}

}


/**
 * Scans the pedestrian_position_grid to find adjacent pedestrians where their movement path cross (X movement) and resolves the conflict by allowing only one pedestrian to move.
 */
void block_X_movement()
{
    bool is_X_movement;

    //Except for the exits, there are no pedestrians at the boundaries of the environment, so no checks are performed there.
    for(int i = 1; i < cli_args.global_line_number - 1; i++) 
    {
        for(int h = 1; h < cli_args.global_column_number - 1; h++)
        {
            int first_pedestrian_id = pedestrian_position_grid[i][h];
            if(first_pedestrian_id > 0) // there is a pedestrian on the cell
            {
                if(pedestrian_set.list[first_pedestrian_id - 1]->state != MOVING  || 
                    pedestrian_set.list[first_pedestrian_id - 1]->in_panic == true)
                    continue;

                // X movements only occur between pedestrians located in vertically or horizontally adjacent cells,
                // so only those cells need to be verified. Due to the scanning method, the [i-1][h] and [i][h-1] cells
                // have already been checked for X movements (or did not require any check), so only the cells located
                // at [i][h+1] and [i+1][h] need to be verified.        

                int second_pedestrian_id = pedestrian_position_grid[i][h + 1];
                if(second_pedestrian_id > 0)  // there is a pedestrian on the cell
                {
                    is_X_movement = are_pedestrian_paths_crossing(pedestrian_set.list[first_pedestrian_id- 1], pedestrian_set.list[second_pedestrian_id - 1]);

                    if(is_X_movement == true)
                    {
                        solve_X_movement(pedestrian_set.list[first_pedestrian_id- 1], pedestrian_set.list[second_pedestrian_id - 1]);
                        continue;
                    }

                }

                second_pedestrian_id = pedestrian_position_grid[i + 1][h];
                if(second_pedestrian_id > 0) // there is a pedestrian on the cell
                {
                    is_X_movement = are_pedestrian_paths_crossing(pedestrian_set.list[first_pedestrian_id- 1], pedestrian_set.list[second_pedestrian_id - 1]);

                    if(is_X_movement == true)
                        solve_X_movement(pedestrian_set.list[first_pedestrian_id- 1], pedestrian_set.list[second_pedestrian_id - 1]);

                }
            }
        }
    }    
}

/**
 *  Pedestrians in MOVING state are moved to their target location (the target Location is copied to the current Location). Upon reaching an exit, their state changes to LEAVING; those already in an exit transition to GOT_OUT. This is how the movement of a pedestrian is done.
 * 
 * @note If the immediate_exit flag is on, the pedestrians go directly from MOVING to GOT_OUT when a exit is reached.
 * 
*/
void apply_pedestrian_movement()
{
    for(int p_index = 0; p_index < pedestrian_set.num_pedestrians; p_index++)
    {
        Pedestrian current_pedestrian = pedestrian_set.list[p_index];
        
        if(current_pedestrian->in_panic == true || current_pedestrian->state == GOT_OUT || current_pedestrian->state == STOPPED)
            continue; // Pedestrian is ignored

        if(current_pedestrian->state == MOVING)
        {
            current_pedestrian->current = current_pedestrian->target;

            if(exits_set.final_floor_field[current_pedestrian->current.lin][current_pedestrian->current.col] == EXIT_VALUE)
            {
                current_pedestrian->state = cli_args.immediate_exit ? GOT_OUT : LEAVING; 
                // Leaving means the pedestrian will remain for a timestep before being removed from the environment.
            }
        }
        else if(current_pedestrian->state == LEAVING)
            current_pedestrian->state = GOT_OUT; // After a timestep in the exit the pedestrian is removed from the environment.
    }
}

/**
 * Verifies if all pedestrians have exited the environment.
 * @return bool, where True indicates that the environment is empty (no pedestrians) and False otherwise.
*/
bool is_environment_empty()
{
    for(int p_index = 0; p_index < pedestrian_set.num_pedestrians; p_index++)
    {
        Pedestrian current_pedestrian = pedestrian_set.list[p_index];
        if(current_pedestrian->state != GOT_OUT)
            return false;
    }

    return true;
}

/**
 * Reset the pedestrian_position_grid and update it with the current position of all pedestrians still in the environment.
*/
void update_pedestrian_position_grid()
{
    initialize_integer_grid(pedestrian_position_grid, cli_args.global_line_number, cli_args.global_column_number, 0);

    for(int p_index = 0; p_index < pedestrian_set.num_pedestrians; p_index++)
    {
        Pedestrian current_pedestrian = pedestrian_set.list[p_index];

        if(current_pedestrian->state == GOT_OUT)
            continue;

        pedestrian_position_grid[current_pedestrian->current.lin][current_pedestrian->current.col] = current_pedestrian->id;
        heatmap_grid[current_pedestrian->current.lin][current_pedestrian->current.col]++;
    }
}

/**
 * Reset the state of all pedestrians to MOVING, except for those in the states GOT_OUT and LEAVING.
*/
void reset_pedestrian_state()
{
    for(int p_index = 0; p_index < pedestrian_set.num_pedestrians; p_index++)
    {
        if(pedestrian_set.list[p_index]->state != GOT_OUT && pedestrian_set.list[p_index]->state != LEAVING)
            pedestrian_set.list[p_index]->state = MOVING;
    }
}

/**
 * Reset the in_panic flag for all pedestrians that aren't in the GOT_OUT state.
*/
void reset_pedestrian_panic()
{
    for(int p_index = 0; p_index < pedestrian_set.num_pedestrians; p_index++)
    {
        if(pedestrian_set.list[p_index]->state != GOT_OUT)
            pedestrian_set.list[p_index]->in_panic = false;
    }
}

/**
 * Reset all pedestrian structures to their original values, i.e., the state is set to MOVING and their current Location is set to the origin Location.
*/
void reset_pedestrians_structures()
{
    initialize_integer_grid(pedestrian_position_grid, cli_args.global_line_number, cli_args.global_column_number, 0);
    
    for(int p_index = 0; p_index < pedestrian_set.num_pedestrians; p_index++)
    {
        Pedestrian current_pedestrian = pedestrian_set.list[p_index];

        current_pedestrian->current.lin = current_pedestrian->origin.lin;
        current_pedestrian->current.col = current_pedestrian->origin.col;
        current_pedestrian->state = MOVING;
        current_pedestrian->in_panic = false;
        pedestrian_position_grid[current_pedestrian->current.lin][current_pedestrian->current.col] = current_pedestrian->id;
    }
}

/* ---------------- ---------------- ---------------- ---------------- ---------------- */
/* ---------------- ---------------- STATIC FUNCTIONS ---------------- ---------------- */
/* ---------------- ---------------- ---------------- ---------------- ---------------- */

/**
 * Creates a new Pedestrian structure based on the given Location.
 * 
 * @note The Pedestrian id is not filled.
 * 
 * @param ped_coordinates New pedestrian coordinates.
 * @return A Null pointer, on error, or a Pedestrian structure if the new Pedestrian is successfully created.
*/ 
static Pedestrian create_pedestrian(Location ped_coordinates)
{
    Pedestrian new_pedestrian = malloc(sizeof(struct pedestrian));
    if(new_pedestrian != NULL)
    {
        new_pedestrian->current = new_pedestrian->origin = ped_coordinates;
        new_pedestrian->target = (Location) {-1, -1};
        new_pedestrian->state = MOVING;
        new_pedestrian->in_panic = false;

        heatmap_grid[ped_coordinates.lin][ped_coordinates.col]++;
    }

    return new_pedestrian;
}

/**
 * Verifies if the paths of the provided pedestrians cross using the reduced straight line formula and intersection of lines.
 * 
 * @param first_pedestrian A Pedestrian.
 * @param second_pedestrian A Pedestrian adjacent to the first_pedestrian.
 * @return bool, where True indicates that the paths cross and False otherwise.
*/
static bool are_pedestrian_paths_crossing(Pedestrian first_pedestrian, Pedestrian second_pedestrian)
{
    if(first_pedestrian->state != MOVING || second_pedestrian->state != MOVING || 
        first_pedestrian->in_panic == true || second_pedestrian->in_panic == true)
        return false;

    reduced_line_equation first_line, second_line;
    // Each straight line struct represents the line containing the segment from the initial to the target location of each pedestrian.

    if(calculate_reduced_line_equation(first_pedestrian->current, first_pedestrian->target, &first_line) == FAILURE ||
       calculate_reduced_line_equation(second_pedestrian->current, second_pedestrian->target, &second_line) == FAILURE)
        return false; //Vertical lines doesn't allow the occurrence of X movement.

    if(first_line.angular_coefficient == 0.0 || second_line.angular_coefficient == 0.0)
        return false; // angular_coefficient equals zero results in a horizontal line, where X movements cannot occur.

    if(first_line.angular_coefficient == second_line.angular_coefficient)
        return false; // Lines are equal or parallel, and therefore X movements cannot occur.

    double intersect_x, intersect_y;

    calculate_intersection_point(first_line, second_line, &intersect_x, &intersect_y);    

    // .lin corresponds to the y-axis and .col corresponds to the x-axis.

    if(first_pedestrian->target.col == intersect_x && first_pedestrian->target.lin == intersect_y)       
        return false; // The intersect point coincides with the target cell coordinates of one pedestrian. This means that both aim to move to the same cell and this characterizes a simples conflict. These conflicts are solved elsewhere. 
    
    if(is_intersection_within_pedestrian_movement(intersect_x, intersect_y, first_pedestrian) == true &&
       is_intersection_within_pedestrian_movement(intersect_x, intersect_y, second_pedestrian) == true)
        return true; // A X movement happens

    return false;
}

/**
 * Calculate the reduced line equation (slope-intercept form) for the line segment beginning at origin and ending at target.
 * 
 * @param origin Coordinates of the origin point of the segment.
 * @param target Coordinates of the target (end) point of the segment.
 * @param line A pointer to a reduced_line_equation struct where the calculated values will be stored.
 * @return Function_Status: FAILURE (0), for vertical lines, or SUCCESS (1).
*/
static Function_Status calculate_reduced_line_equation(Location origin, Location target, reduced_line_equation* line)
{
    // .lin corresponds to the y-axis and .col corresponds to the x-axis.

    if(target.col == origin.col)
        return FAILURE;
        // Infinite angular coefficient (denominator equals zero: xf - xi). This results in a vertical line that cannot be represented in the reduced form. 

    // angular_coefficient = (yf - yi) / (xf - xi)
    line->angular_coefficient = (target.lin - origin.lin) / (target.col - origin.col);
     
    // linear_coefficient = yi - m * xi
    line->linear_coefficient = origin.lin - line->angular_coefficient * origin.col;

    return SUCCESS;
}

/**
 * Calculate the x and y coordinates of the intersection point of the provided lines.
 * 
 * @note If the provided lines are parallel, both x and y will be set to zero.
 * 
 * @param first_line A straight line represented in the slope-intercept form.
 * @param second_line A straight line represented in the slope-intercept form.
 * @param x A pointer to a double, where the calculated interception point x will be stored.
 * @param y A pointer to a double, where the calculated interception point y will be stored.
*/
static void calculate_intersection_point(reduced_line_equation first_line, reduced_line_equation second_line, double *x, double *y)
{
    /*
        (1): y = m1 * x + n1
        (2): y = m2 * x + n2

        m1 * x + n1 = m2 * x + n2 --> m1*x - m2*x = n2 - n1 --> x(m1 - m2) = n2 - n1
        x = (n2 - n1) / (m1 - m2) --> Expression to calculate the x-axis intersection point.
                                      With the x-axis point calculated, its possible to derive the y-axis intersection point.
    */

    if(first_line.angular_coefficient == second_line.angular_coefficient)
    {
        *x = *y = 0;
        return;
    }

    *x = (second_line.linear_coefficient - first_line.linear_coefficient) / 
        (first_line.angular_coefficient - second_line.angular_coefficient);
    *y = first_line.angular_coefficient * (*x) + first_line.linear_coefficient;
}

/**
 * Verifies if the point (x_coordinate, y_coordinate) is within a the line segment defined by the current and target locations of the given pedestrian.
 * 
 * @param x_coordinate A double, representing the x-axis coordinate.
 * @param y_coordinate A double, representing the y-axis coordinate.
 * @param first_pedestrian A Pedestrian.
 * @return bool, where True indicates that the point is within the line segment.
*/
static bool is_intersection_within_pedestrian_movement(double x_coordinate, double y_coordinate, Pedestrian pedestrian)
{
    return  x_coordinate > fmin(pedestrian->current.col, pedestrian->target.col) && 
            x_coordinate < fmax(pedestrian->current.col, pedestrian->target.col) && 
            y_coordinate > fmin(pedestrian->current.lin, pedestrian->target.lin) && 
            y_coordinate < fmax(pedestrian->current.lin, pedestrian->target.lin);
}

/**
 * Decides which of the given pedestrians will be allowed to move.
 * 
 * @param first_pedestrian A Pedestrian involved in a X movement.
 * @param second_pedestrian A pedestrian involved in an X movement.
*/
static void solve_X_movement(Pedestrian first_pedestrian, Pedestrian second_pedestrian)
{
    int sorted_num = rand() % 100;

    if(sorted_num < 50)
        second_pedestrian->state = STOPPED;
    else
        first_pedestrian->state = STOPPED;

    if(cli_args.show_debug_information)
        printf("X Movement between %d and %d --> %d.\n", first_pedestrian->id, second_pedestrian->id, 
                                                         sorted_num < 50 ? first_pedestrian->id : second_pedestrian->id);
}