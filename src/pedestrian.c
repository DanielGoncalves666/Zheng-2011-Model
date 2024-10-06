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

#include"../headers/exit.h"
#include"../headers/dynamic_field.h"
#include"../headers/grid.h"
#include"../headers/pedestrian.h"
#include"../headers/cli_processing.h"
#include"../headers/shared_resources.h"
#include"../headers/static_field.h"
#include"../headers/fire_dynamics.h"
#include"../headers/fire_field.h"

#include"../headers/printing_utilities.h"

Int_Grid pedestrian_position_grid = NULL; // Grid containing pedestrians at their respective positions.

typedef struct cell_conflict{
    int num_pedestrians;
    int pedestrian_ids[8];
    int pedestrian_allowed;
}cell_conflict;

Pedestrian_Set pedestrian_set = {NULL,0,0};

static Pedestrian create_pedestrian(Location ped_coordinates);
static void calculate_transition_probabilities(Pedestrian current_pedestrian);
static Location transition_selection(Pedestrian current_pedestrian);
static Location calculate_inertia_mask(Location previous, Location current);
static bool evaluate_pedestrian_vision(Pedestrian current_pedestrian);
static bool is_vision_blocked(Location origin, Location destination);
static bool is_pedestrian_dead(Pedestrian current_pedestrian);

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

        if(is_pedestrian_dead(current_pedestrian))
        {
            current_pedestrian->state = DEAD;
            pedestrian_set.num_dead_pedestrians++;
        }

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

    int random_result;
    for(int conflict_index = 0; conflict_index < num_conflicts; conflict_index++)
    {
        Cell_Conflict current_conflict = &(pedestrian_conflicts[conflict_index]);

        if(probability_test(cli_args.mu)) // The movement to all pedestrian in the conflict has been denied
            random_result = -1; // Since random_result has been assigned as -1, all pedestrian will have their states changed to STOPPED.
        else
        {
            random_result = roulette_wheel_selection((double *) &probabilities, current_conflict->num_pedestrians,   current_conflict->num_pedestrians);
        }

        current_conflict->pedestrian_allowed = random_result == -1 ? -1 : current_conflict->pedestrian_ids[random_result];
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
        
        if(current_pedestrian->state == GOT_OUT || current_pedestrian->state == STOPPED || current_pedestrian->state == DEAD)
            continue; // Pedestrian is ignored

        if(current_pedestrian->state == MOVING)
        {
            current_pedestrian->previous = current_pedestrian->current;
            current_pedestrian->current = current_pedestrian->target;

            if(exits_only_grid[current_pedestrian->current.lin][current_pedestrian->current.col] == EXIT_CELL)
            {
                current_pedestrian->state = cli_args.immediate_exit ? GOT_OUT : LEAVING; 
                // Leaving means the pedestrian will remain for a timestep before being removed from the environment.
            }
        }
        else if(current_pedestrian->state == LEAVING)
        {
            current_pedestrian->state = GOT_OUT; // After a timestep in the exit the pedestrian is removed from the environment.
        }
    }
}

/**
 * Verifies if all alive pedestrians have exited.
 * @return bool, where True indicates that the environment is empty (no alive pedestrians) and False otherwise.
*/
bool is_environment_empty()
{
    for(int p_index = 0; p_index < pedestrian_set.num_pedestrians; p_index++)
    {
        Pedestrian current_pedestrian = pedestrian_set.list[p_index];
        if(current_pedestrian->state != GOT_OUT && current_pedestrian->state != DEAD)
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

        if(current_pedestrian->state == GOT_OUT || current_pedestrian->state == DEAD)
            continue;

        pedestrian_position_grid[current_pedestrian->current.lin][current_pedestrian->current.col] = current_pedestrian->id;
        heatmap_grid[current_pedestrian->current.lin][current_pedestrian->current.col]++;
    }
}

/**
 * Reset the state of all pedestrians to MOVING, except for those in the states GOT_OUT, LEAVING or DEAD.
*/
void reset_pedestrian_state()
{
    for(int p_index = 0; p_index < pedestrian_set.num_pedestrians; p_index++)
    {
        Pedestrian current_pedestrian = pedestrian_set.list[p_index];

        if(current_pedestrian->state != GOT_OUT && current_pedestrian->state != LEAVING && current_pedestrian->state != DEAD)
            current_pedestrian->state = MOVING;
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
static void calculate_transition_probabilities(Pedestrian current_pedestrian)
{
    double alpha = 1;

    double normalization_value = 0; // The N value in the formula

    Double_Grid static_field = evaluate_pedestrian_vision(current_pedestrian) ? exits_set.aux_static_grid : exits_set.static_floor_field;
    // Necessário calcular uma grid de distâncias
    // Ou calcular apenas as distancias das celulas na vizinhança

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            if(i != 1 && j != 1) // Ignore the diagonals, since all their probabilities is 0.
                continue;

            int lin = current_pedestrian->current.lin + i - 1;
            int col = current_pedestrian->current.col + j - 1;

            if(! is_within_grid_lines(lin) || 
                ! is_within_grid_columns(col) ||
                is_cell_with_fire((Location) {lin, col}) ||
                static_field[lin][col] == IMPASSABLE_OBJECT ||
                risky_cells_grid[lin][col] == DANGER_CELL) 
            {
                // Verifying fire and obstacles before performing any part of the calculation helps avoid unnecessary computations.
                current_pedestrian->probabilities[i][j] = 0;

                continue;
            }

            // Static floor field
            current_pedestrian->probabilities[i][j] = exp(cli_args.ks * static_field[lin][col]);

            // Dynamic floor field
            current_pedestrian->probabilities[i][j] *= exp(cli_args.kd * exits_set.dynamic_floor_field[lin][col]); 

            // Fire floor field
            if(risky_cells_grid[lin][col] == NON_RISKY_CELLS) // If its a risky cell (danger cells have already been verified out) the pedestrian ignores the influence of the fire and this code isn't run.
            {
                if(exits_set.distance_to_exits_grid[lin][col] < cli_args.risk_distance)
                    alpha = cli_args.fire_alpha;
                else
                    alpha = 1;

                current_pedestrian->probabilities[i][j] /= exp(cli_args.kf * alpha * exits_set.fire_floor_field[i][j]);
            }

            if(! (i == 1 && j == 1)) // Ignores when it is the pedestrian's cell.
                current_pedestrian->probabilities[i][j] *= pedestrian_position_grid[lin][col] > 0 ? 0 : 1; // Multiply by 0 if cell is occupied

            normalization_value += current_pedestrian->probabilities[i][j];  
        }
    }

    if( !are_same_coordinates(current_pedestrian->previous, current_pedestrian->current))
    {
        // If the pedestrian has moved on the previous timestep
        Location inertia_mask = calculate_inertia_mask(current_pedestrian->previous, current_pedestrian->current);
        double former_probability = current_pedestrian->probabilities[inertia_mask.lin + 1][inertia_mask.col + 1];

        current_pedestrian->probabilities[inertia_mask.lin + 1][inertia_mask.col + 1] *= cli_args.omega;
        // If the correspondent cell is occupied by anything, omega will have no effect (since is a multiplication with 0).

        normalization_value += current_pedestrian->probabilities[inertia_mask.lin + 1][inertia_mask.col + 1] - former_probability;
    }

    if(! normalization_value == 0) // Avoids 0 to the power of -1.
        normalization_value = pow(normalization_value, -1);

    for(int i = 0; i < 3; i++)
    {
        for(int j = 0; j < 3; j++)
        {
            current_pedestrian->probabilities[i][j] *= normalization_value;
        }
    }

    if(current_pedestrian->id == 8)
    {
        printf("%d %d\n", current_pedestrian->current.lin, current_pedestrian->current.col);
        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3; j++)
            {
                printf("%.3lf ", current_pedestrian->probabilities[i][j]);
            }
            printf("\n");
        }
        fflush(stdout);
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
static Location transition_selection(Pedestrian current_pedestrian)
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

    // If no cell is selected due to rounding errors or all probabilities being zero, the pedestrian will stay in the current cell.
    return current_coordinates; 
}

/**
 * Calculates the inertia mask for the given locations. An inertia mask is a location modifier that, when added to the coordinates of the current location, yields the coordinates of the next cell along the same path of movement from the previous to the current location.
 * 
 * @param previous The former cell coordinates
 * @param current The current cell coordinates
 * 
 * @note Works for movements in the diagonals.
 * 
 * @return A location, containing the calculated inertia mask.
 */
static Location calculate_inertia_mask(Location previous, Location current)
{
    return (Location) {current.lin - previous.lin, current.col - previous.col};
}

/**
 * Verify if the given pedestrian has the vision of any non-blocked exit cell obstructed. If true, a static floor field without the affected exit cells is calculated and stored in the aux_static_grid.
 * 
 * @param current_pedestrian The pedestrian whose vision will be evaluated.
 * @return A bool, indicating if the pedestrian's view of any exit cell is obstructed (true) or not (false).
 */
static bool evaluate_pedestrian_vision(Pedestrian current_pedestrian)
{
    Location current_loc = current_pedestrian->current;
    Location *exit_cell_coordinates = NULL;
    int num_exit_cells = 0;

    bool vision_blocked = false; // Indicates whether the pedestrian's view of ANY exit is obstructed.

    for(int exit_index = 0; exit_index < exits_set.num_exits; exit_index++) 
    {
        Exit current_exit = exits_set.list[exit_index];

        if(current_exit->is_blocked_by_fire)
            continue;

        for(int cell_index = 0; cell_index < current_exit->width; cell_index++)
        {
            Location current_cell = current_exit->coordinates[cell_index];

            if(is_vision_blocked(current_loc, current_cell))
            {
                vision_blocked = true;
                continue;
            }
            
            exit_cell_coordinates = realloc(exit_cell_coordinates, sizeof(Location) * (num_exit_cells + 1));
            if(exit_cell_coordinates == NULL)
            {
                fprintf(stderr, "Failure in the realloc of the exit_cells_coordinates list (evaluate_pedestrian_vision).\n");
                return NULL;
            }

            exit_cell_coordinates[num_exit_cells] = current_cell;
            num_exit_cells++;
        }
    }

    calculate_zheng_static_field(exit_cell_coordinates, num_exit_cells, exits_set.aux_static_grid);

    free(exit_cell_coordinates);

    return vision_blocked;
}

/**
 * Verifies if the pedestrian in the origin location doesn't have a clean vision of the exit at destination.
 * 
 * @note Utilizes the Bresenham Line Algorithm.
 * 
 * @param origin The Location of the pedestrian.
 * @param destination The location of the exit.
 * 
 * @return bool, where True indicates that the pedestrian doesn't have a clean vision of the exit, or False if he does see the exit.
 */
static bool is_vision_blocked(Location origin, Location destination)
{
    // Sugestão: Pedestre verificar a célula na frente da porta, invés da porta

    int x1 = origin.col, y1 = origin.lin;
    int x2 = destination.col, y2 = destination.lin; // Just to facilitate the operations

    int dx = x2 - x1; // The difference in the x axis. 
    int dy = y2 - y1; // The difference in the y axis.

    int error; // Also know as the decision variable

    int x_step = 1; // The direction the algorithm will step in the x-axis
                    // 1: --> , -1: <--
    int y_step = 1; // The direction the algorithm will step in the y-axis
                    // 1: down-top, -1: top-down

    int y = y1, x = x1; // The coordinates used to iterate through the line
    
    if(is_cell_with_fire((Location) {y1, x1}))
        return true;

    if(dy < 0)
    {
        y_step = -1; // top-down
        dy = -dy;
    }
    
    if(dx < 0)
    {
        x_step = -1; // <--
        dx = -dx;
    }

    int ddx = 2 * dx;
    int ddy = 2 * dy; // Both used to update the error variable
    if(ddx >= ddy) //The algorithm uses the x-axis as the main direction
    {
        error = ddy - dx;

        for(int i = 0; i < dx; i++)
        {
            x += x_step; // This forces the algorithm to start at the second point

            if(error >  0)
            {
                y += y_step;
                error -= ddx;
            }
            error += ddy;

            if(is_cell_with_fire((Location) {y, x}))
                return true;
        }
    }
    else // The algorithm uses the y-axis as the main direction
    {
        error = ddx - dy;

        for(int i = 0; i < dy; i++)
        {
            y += y_step;

            if(error > 0)
            {
                x += x_step;
                error -= ddy;
            }
            error += ddx;

            if(is_cell_with_fire((Location) {y, x}))
                return true;
        }
    }

    return false;
}

/**
 * Verify if the given pedestrian is dead (it is on a cell with fire).
 * 
 * @param current_pedestrian Pedestrian for which the verification will be performed.
 * 
 * @return bool, where True indicates that the pedestrian is dead, or False otherwise.
 */
static bool is_pedestrian_dead(Pedestrian current_pedestrian)
{
    Location current_location = current_pedestrian->current;

    return fire_grid[current_location.lin][current_location.col] == FIRE_CELL;
}