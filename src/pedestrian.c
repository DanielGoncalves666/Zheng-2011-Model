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
#include"../headers/dynamic_field.h"
#include"../headers/grid.h"
#include"../headers/pedestrian.h"
#include"../headers/cli_processing.h"
#include"../headers/shared_resources.h"

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
void calculate_transition_probabilities(Pedestrian current_pedestrian);
Location transition_selection(Pedestrian current_pedestrian);
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

    if(fill_integer_grid(pedestrian_position_grid, cli_args.global_line_number, cli_args.global_column_number, 0) == FAILURE)
        return FAILURE;

    for(int p_index = 0; p_index < num_pedestrians_to_insert; p_index++)
    {

        int line = (int) rand_within_limits(1, cli_args.global_line_number - 1);
        int column = (int) rand_within_limits(1, cli_args.global_column_number - 1);
        // If any of the drawn number equals the final number of the interval, the coordinates will correspond to a cell occupied by a wall or door. These cases are handled by the is_cell_empty function call bellow.

        // If the cell at the draw coordinates is empty, the pedestrian is immediately inserted there. But, if its not, the pedestrian will be place in the next available empty cell.
        // If no empty cell is found when the scan reaches the bottom right of the environment, the search will restart at the top left.

        bool next_pedestrian = false; // Indicates if the pedestrian has already been inserted and that the for loops bellow should end.  
        bool already_looping = false; // Indicates if the search for an empty cell has already looped around.
        for(; line < cli_args.global_line_number - 1; line++)
        {
            for(; column < cli_args.global_column_number - 1; column++)
            {       
                if(is_cell_empty((Location) {line, column}) == true)
                {
                    if( add_new_pedestrian((Location) {line, column}) == FAILURE)
                        return FAILURE;

                    pedestrian_position_grid[line][column] = pedestrian_set.list[pedestrian_set.num_pedestrians - 1]->id;

                    next_pedestrian = true;
                    break;
                }
            }

            if(next_pedestrian)
                break;

            column = 1;

            if(line + 1 == cli_args.global_line_number - 1)
            {
                if(already_looping)
                {
                    fprintf(stderr, "There is not enough empty space to accommodate the specified number of pedestrians.\n");
                    return FAILURE;
                }
                
                line = 1; // From the draw cell until the end of the environment there is no empty cell, so is necessary to loop around.
                already_looping = true;
            }
        }
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
 * Determines the destination cell for each pedestrian.
*/
void evaluate_pedestrians_movements()
{
    for(int p_index = 0; p_index < pedestrian_set.num_pedestrians; p_index++)
    {
        Pedestrian current_pedestrian = pedestrian_set.list[p_index];

        if(current_pedestrian->state != MOVING)
            continue;

        calculate_transition_probabilities(current_pedestrian);
        Location destination_cell = transition_selection(current_pedestrian);

        current_pedestrian->target = destination_cell;
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

        if(current_pedestrian->state != MOVING)
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
    static double probabilities[] = {1, 1, 1, 1, 1, 1, 1, 1};
    // The maximum number of conflicts (when considering diagonal movements) is 8.

    if(pedestrian_conflicts == NULL && num_conflicts > 0)
    {
        fprintf(stderr, "Null pointer received at solve_pedestrian_conflicts when a valid pointer was expected.\n");
        return FAILURE;
    }

    for(int conflict_index = 0; conflict_index < num_conflicts; conflict_index++)
    {
        Cell_Conflict current_conflict = &(pedestrian_conflicts[conflict_index]);

        int random_result = roulette_wheel_selection((double *) &probabilities, current_conflict->num_pedestrians,   
                                                    current_conflict->num_pedestrians);

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
                if(pedestrian_set.list[first_pedestrian_id - 1]->state != MOVING)
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

        if(current_pedestrian->state != GOT_OUT && !cli_args.velocity_density_field)
            increase_particle_at(current_pedestrian->current); // When the dynamic field is defined as a particle density field, increases the particle count for all cells occupied by pedestrians (not only for those which the pedestrian is moving away).
        
        if(current_pedestrian->state == GOT_OUT || current_pedestrian->state == STOPPED)
            continue; // Pedestrian is ignored

        if(current_pedestrian->state == MOVING)
        {
            if(! are_same_coordinates(current_pedestrian->current, current_pedestrian->target))
            {
                // In case the pedestrian didn't move, the previous cell is preserved (since it isn't updated).
                // This guarantees the pedestrian will not get confused with its own trace.

                current_pedestrian->previous = current_pedestrian->current;

                // When the dynamic field has been defined as a particle density field, the increase in the cell particles happens only in the beginning of each timestep, considering the current location of each pedestrian and not when (if) they move.
                if(cli_args.velocity_density_field)
                    increase_particle_at(current_pedestrian->current);
            }

            current_pedestrian->current = current_pedestrian->target;

            if(exits_only_grid[current_pedestrian->current.lin][current_pedestrian->current.col] == EXIT_CELL)
            {
                current_pedestrian->state = cli_args.immediate_exit ? GOT_OUT : LEAVING; 
                // Leaving means the pedestrian will remain for a timestep before being removed from the environment.
            }
        }
        else if(current_pedestrian->state == LEAVING)
        {
            if(cli_args.velocity_density_field)
                increase_particle_at(current_pedestrian->current);

            current_pedestrian->state = GOT_OUT; // After a timestep in the exit the pedestrian is removed from the environment.
        }
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
    fill_integer_grid(pedestrian_position_grid, cli_args.global_line_number, cli_args.global_column_number, 0);

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
 * Reset all pedestrian structures to their original values, i.e., the state is set to MOVING and their current Location is set to the origin Location.
*/
void reset_pedestrians_structures()
{
    fill_integer_grid(pedestrian_position_grid, cli_args.global_line_number, cli_args.global_column_number, 0);
    
    for(int p_index = 0; p_index < pedestrian_set.num_pedestrians; p_index++)
    {
        Pedestrian current_pedestrian = pedestrian_set.list[p_index];

        current_pedestrian->previous = current_pedestrian->origin;
        current_pedestrian->current = current_pedestrian->origin;
        current_pedestrian->state = MOVING;
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
    Pedestrian new_pedestrian = calloc(1, sizeof(struct pedestrian));
    if(new_pedestrian != NULL)
    {
        new_pedestrian->current = new_pedestrian->previous = new_pedestrian->origin = ped_coordinates;
        new_pedestrian->target = (Location) {-1, -1};
        new_pedestrian->state = MOVING;
        // probabilities are already all set to 0.
    
        heatmap_grid[ped_coordinates.lin][ped_coordinates.col]++;
    }

    return new_pedestrian;
}

/**
 * Calculates the transition probabilities for the neighborhood of the provided pedestrian.
 * 
 * @param current_pedestrian Pedestrian for which the transition probabilities will be calculated.
 */
void calculate_transition_probabilities(Pedestrian current_pedestrian)
{
    int lin = current_pedestrian->current.lin;
    int col = current_pedestrian->current.col;

    double normalization_value = 0; // The N value in the formula

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            if(i == 1 || j == 1) // Ignore the diagonals, since they are all 0.
            {
                if(! is_within_grid_lines(lin + (i - 1)) || 
                   ! is_within_grid_columns(col + (j - 1)))
                {
                    current_pedestrian->probabilities[i][j] = 0;
                    continue;
                }

                int dynamic_floor_field_value = exits_set.dynamic_floor_field[lin + i - 1][col + j - 1];

                if(cli_args.ignore_latest_self_trace)
                {
                    if(! are_same_coordinates(current_pedestrian->origin, current_pedestrian->previous))
                    {
                        // Guarantees that the pedestrian has already moved from its original position.

                        if(are_same_coordinates(current_pedestrian->previous, (Location) {lin + (i - 1), col + (j - 1)}))
                        {
                            // The cell of the neighborhood which the transition probability is being calculated is the cell that the pedestrian was previously located.

                            if(dynamic_floor_field_value > 0)
                                dynamic_floor_field_value--;
                        }
                    }
                }

                // Static floor field
                current_pedestrian->probabilities[i][j] = exp(cli_args.ks * exits_set.static_floor_field[lin + i - 1][col + j - 1]);
                // Dynamic floor field
                current_pedestrian->probabilities[i][j] *= exp(cli_args.kd * dynamic_floor_field_value); 

                if(! (i == 1 && j == 1)) // Ignores when is the pedestrian's cell.
                    current_pedestrian->probabilities[i][j] *= 1 - (pedestrian_position_grid[lin + i - 1][col + j - 1] > 0 ? 1 : 0); // Multiply by 0 if cell is occupied
                
                current_pedestrian->probabilities[i][j] *= exits_set.static_floor_field[lin + i - 1][col + j - 1] == WALL_CELL ? 0 : 1; // Multiply by 0 if cell has an obstacle.
            
                normalization_value += current_pedestrian->probabilities[i][j];
            }

        }
    }

    normalization_value = pow(normalization_value, -1);

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            current_pedestrian->probabilities[i][j] *= normalization_value;
        }
    }
}

/**
 * Simulates a roulette wheel selection to determine the destination cell for a pedestrian's movement,
 * based on his transition probabilities for the neighboring cells.
 * 
 * @param current_pedestrian The pedestrian for whom a destination cell will be selected, based on 
 *                           his transition probabilities for the neighboring cells.
 * 
 * @return The coordinates of the selected destination cell.
 */
Location transition_selection(Pedestrian current_pedestrian)
{
    float draw_value = 0;
    draw_value = rand_within_limits(0,1);
    
    Location current_coordinates = current_pedestrian->current;

    double total = 0;
    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            if(current_pedestrian->probabilities[i][j] == 0)
                continue;

            total += current_pedestrian->probabilities[i][j];

            if(draw_value <= total + TOLERANCE)
                return (Location) {current_coordinates.lin + (i - 1), current_coordinates.col + (j - 1)};
        }
    }

    return current_coordinates; // If no cell is selected due to rounding errors, the probability corresponding to the pedestrian's current cell will be chosen.
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
    if(first_pedestrian->state != MOVING || second_pedestrian->state != MOVING)
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
    double sorted_value = rand_within_limits(0,1);
    
    if(sorted_value <= 0.5 + TOLERANCE)
        second_pedestrian->state = STOPPED;
    else
        first_pedestrian->state = STOPPED;

    if(cli_args.show_debug_information)
        printf("X Movement between %d and %d --> %d.\n", first_pedestrian->id, second_pedestrian->id, 
                                                         sorted_value < 0.5 + TOLERANCE? first_pedestrian->id : second_pedestrian->id);
}