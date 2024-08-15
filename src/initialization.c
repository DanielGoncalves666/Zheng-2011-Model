/* 
   File: initialization.c
   Author: Daniel Gonçalves
   Date: 2023-10-15
   Description: This module implements functions responsible for opening environment, output, and auxiliary files, reading data from these files, allocating integer grids used by the program, and generating the environment if necessary.
*/


#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<stdbool.h>
#include<time.h>

#include"../headers/grid.h"
#include"../headers/exit.h"
#include"../headers/pedestrian.h"
#include"../headers/initialization.h"
#include"../headers/cli_processing.h"
#include"../headers/shared_resources.h"

const char *environment_path = "environments/";
const char *auxiliary_path = "auxiliary/";
const char *output_path = "output/";

static Function_Status open_environment_file(FILE **environment_file);
static Function_Status symbol_processing(char read_char, Location coordinates);

/**
 * Opens the auxiliary file in read mode.  
 * 
 * @param auxiliary_file Pointer to the FILE structure that will hold the file descriptor.
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
*/
Function_Status open_auxiliary_file(FILE **auxiliary_file)
{
    char complete_path[500] = "";
    
    if( origin_uses_auxiliary_data() == true)
    {
        sprintf(complete_path,"%s%s",auxiliary_path,cli_args.auxiliary_filename);

        *auxiliary_file = fopen(complete_path,"r");
        if(*auxiliary_file == NULL)
        {
            fprintf(stderr, "It was not possible to open the auxiliary file.\n");
            return FAILURE;
        }
    }

    return SUCCESS;
}

/**
 * Opens the output file in write mode.
 * 
 * @note If no file name is provided with the -o option, a name is generated automatically.
 * 
 * @param output_file Pointer to the FILE structure that will hold the file descriptor.
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
*/
Function_Status open_output_file(FILE **output_file)
{
    char complete_path[300] = "";
    char date_time[51];

    if(cli_args.write_to_file)
    {
        // no filename was provided
        if(strcmp(cli_args.output_filename, "") == 0)
        {
            char *output_type_name;
            if(cli_args.output_format == OUTPUT_VISUALIZATION)
                output_type_name = "visual";
            else if(cli_args.output_format == OUTPUT_TIMESTEPS_COUNT)
                output_type_name = "evacuation_time";
            else if(cli_args.output_format == OUTPUT_HEATMAP)
                output_type_name = "heatmap";
            
            time_t current_time = time(NULL);
	        struct tm * time_information = localtime(&current_time);
	
	        strftime(date_time,50,"%F_%Z_%T",time_information);

            sprintf(complete_path,"%s%s-%s-%s.txt", output_path, output_type_name, 
                    cli_args.environment_filename,date_time);
        }
        else
            sprintf(complete_path,"%s%s",output_path,cli_args.output_filename);


        *output_file = fopen(complete_path,"w");
        if(*output_file == NULL)
        {
            fprintf(stderr, "It was not possible to open the output file.\n");
            return FAILURE;
        }
    }
    else
        *output_file = stdout;

    return SUCCESS;
}

/**
 * Allocates the integer grids necessary for the program (environment, exits, pedestrian and heatmap grids).
 *  
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
*/
Function_Status allocate_grids()
{
    environment_only_grid = allocate_integer_grid(cli_args.global_line_number, cli_args.global_column_number);
    exits_only_grid = allocate_integer_grid(cli_args.global_line_number, cli_args.global_column_number);
    pedestrian_position_grid = allocate_integer_grid(cli_args.global_line_number, cli_args.global_column_number);
    heatmap_grid = allocate_integer_grid(cli_args.global_line_number, cli_args.global_column_number);
    if(environment_only_grid == NULL || exits_only_grid == NULL || pedestrian_position_grid == NULL || heatmap_grid == NULL)
    {
        fprintf(stderr,"Failure during allocation of the integer grids with dimensions: %d x %d.\n", cli_args.global_line_number, cli_args.global_column_number);
        return FAILURE;
    }

    return SUCCESS;
}

/**
 * Loads the environment stored in the file provided by the --env-file option.
 * 
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
*/
Function_Status load_environment()
{
    FILE *environment_file = NULL;

    if(open_environment_file(&environment_file) == FAILURE)
        return FAILURE;

    if( fscanf(environment_file,"%d %d", &(cli_args.global_line_number), &(cli_args.global_column_number)) != 2)
    {
        fprintf(stderr, "Environment dimensions weren't found in the first line of the file.\n");
        return FAILURE;
    }

    if(allocate_grids() == FAILURE)
        return FAILURE;

    if(fill_integer_grid(pedestrian_position_grid, cli_args.global_line_number, cli_args.global_column_number, 0) == FAILURE)
        return FAILURE;

    char read_char = '\0';
    fscanf(environment_file,"%c",&read_char);// responsible for eliminating the '\n' after the environment dimensions.
    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        int h = 0;
        for(; h <= cli_args.global_column_number; h++)
        {
            if(fscanf(environment_file,"%c",&read_char) == EOF)
                break;

            if(h == cli_args.global_column_number && read_char != '\n')
            {
                // The end of a line should have been reached
                fprintf(stderr,"Line %d has more columns than the extracted column number.\n", i);
                return FAILURE;
            }

            if(read_char == '\n')
                break;

            if( symbol_processing(read_char,(Location){i,h}) == FAILURE)
                return FAILURE;
        }

        if( h < cli_args.global_column_number)
        {
            fprintf(stderr,"Line %d has less columns than the extracted column number.\n", i);
            return FAILURE;
        }
    }

    fclose(environment_file);

    return SUCCESS;
}

/**
 * Generates a rectangular environment with dimensions specified by global_line_number and global_column_number.The edges will have walls, while the rest of the room will be empty.
 * 
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
*/
Function_Status generate_environment()
{
    if(allocate_grids() == FAILURE)
        return FAILURE;

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int h = 0; h < cli_args.global_column_number; h++)
        {
            if(i > 0 && i < cli_args.global_line_number - 1 && h > 0 && h < cli_args.global_column_number - 1)
                environment_only_grid[i][h] = EMPTY_CELL;
            else
                environment_only_grid[i][h] = WALL_CELL;
        }
    }

    return SUCCESS;
}

/**
 * Determines the number of simulation sets (number of non-empty lines) present in the auxiliary file.
 * 
 * @param auxiliary_file File where the simulation sets are stored.
 * @return A non-negative integer, indicating the number of simulation sets, or -1, in failure.
*/
int extract_simulation_set_quantity(FILE *auxiliary_file)
{
    if(auxiliary_file == NULL)
        return -1;

    char current_char = '\n', previous_char = '\n';
    int count = 0; 

    while(1)
    {
        previous_char = current_char;
        current_char = fgetc(auxiliary_file);

        if(current_char == '\n')
        {
            if(previous_char == '\n')
                continue; // ignore empty lines

            count++;
        }

        if(current_char == EOF)
        {
            if(previous_char != '\n')
                count++; //Count the last line that hasn't ended with a newline

            if(feof(auxiliary_file))
                break;

            if(ferror(auxiliary_file))
            {
                fprintf(stderr, "Error while reading the auxiliary file.");
                return -1;
            }
        }
    }

    fseek(auxiliary_file, 0, SEEK_SET);

    return count;
}

/**
 * Read the next line of the provided auxiliary file and extract the exits coordinates from it, adding them to the environment.
 * 
 * @param auxiliary_file File where the simulation sets are stored.
 * @param exit_number Pointer to a integer, where will be stored the number of exits in the simulation set.
 * 
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
*/
Function_Status get_next_simulation_set(FILE *auxiliary_file, int *exit_number)
{
    Location temp_coordinates;
    int exit_count = 0; // Number of extracted exits.
    char read_char = '\0';
   
    bool new_exit = true; // True for a new exit, False for an expansion over the last new exit.

    if( fill_integer_grid(exits_only_grid, cli_args.global_line_number, cli_args.global_column_number, EMPTY_CELL) == FAILURE)
        return FAILURE; // Resets the exits_only_grid, allowing for it to be reused.

    while(1)
    {
        int returned_value = fscanf(auxiliary_file,"%d %d %c ",&temp_coordinates.lin,&temp_coordinates.col,&read_char);

        if( returned_value == EOF)
            break; // All simulation sets were processed.

        if( returned_value != 3)
        {
            fprintf(stderr, "Failure while reading the auxiliary file for exit coordinates. Verify if the syntax is being correctly followed.\n");
            return FAILURE;
        }

        if(new_exit == true)
        {
            exit_count++;
            if( add_new_exit(temp_coordinates) == FAILURE)
                return FAILURE;
        }
        else
        {
            if( expand_exit(exits_set.list[exits_set.num_exits - 1],temp_coordinates) == FAILURE)
                return FAILURE;
        }

        exits_only_grid[temp_coordinates.lin][temp_coordinates.col] = EXIT_CELL;

        if(read_char == '+')
            new_exit = false;
        else if(read_char == ',')
            new_exit = true;
        else if(read_char == '.')
            break;
        else
        {
            fprintf(stderr, "Unknow symbol in the auxiliary file.\n");
            return FAILURE;
        }
    }

    *exit_number = exit_count;

    return SUCCESS;
}

/**
 * Simply counts the number of empty cells in the environment (i.e, cells not occupied by walls or obstacles).
 * 
 * @return An integer, representing the number of empty cells.
 */
int count_number_empty_cells()
{
    int count = 0;

    for(int i = 0; i < cli_args.global_line_number; i++)
    {
        for(int j = 0; j < cli_args.global_column_number; j++)
        {
            if(environment_only_grid[i][j] == EMPTY_CELL)
                count++;
        }
    }

    return count;
}

/* ---------------- ---------------- ---------------- ---------------- ---------------- */
/* ---------------- ---------------- STATIC FUNCTIONS ---------------- ---------------- */
/* ---------------- ---------------- ---------------- ---------------- ---------------- */

/**
 * Opens the environment file in read mode.
 * 
 * @param environment_file Pointer to the FILE structure that will hold the file descriptor.
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
*/
static Function_Status open_environment_file(FILE **environment_file)
{
    char complete_path[300] = "";
    sprintf(complete_path,"%s%s",environment_path,cli_args.environment_filename);

    *environment_file = fopen(complete_path, "r");
    if(*environment_file == NULL)
    {
        fprintf(stderr,"It was not possible to open the environment file: %s.\n",cli_args.environment_filename);
        return FAILURE;
    }

    return SUCCESS;
}

/**
 * Process the symbol (character) read from the environment file.
 * 
 * @param read_char The last symbol read.
 * @param coordinates The coordinates of the symbol in the environment grid.
 * 
 * @return Function_Status: FAILURE (0) or SUCCESS (1).
*/
static Function_Status symbol_processing(char read_char, Location coordinates)
{
    switch(read_char)
    {
        case '#':
            environment_only_grid[coordinates.lin][coordinates.col] = WALL_CELL;
            break;
        case '_':
            if(origin_uses_static_exits() == true)
            {
                if(add_new_exit(coordinates) == FAILURE)
                    return FAILURE;
                
                environment_only_grid[coordinates.lin][coordinates.col] = WALL_CELL;
                exits_only_grid[coordinates.lin][coordinates.col] = EXIT_CELL;
            }
            else
                environment_only_grid[coordinates.lin][coordinates.col] = WALL_CELL;
                // If a exit is located in the middle of the environment a Wall is still put there.
            break;
        case '.':
            environment_only_grid[coordinates.lin][coordinates.col] = EMPTY_CELL;
            break;
        case 'p':
        case 'P':
            if(origin_uses_static_pedestrians() == true)
            {
                if( add_new_pedestrian(coordinates) == FAILURE)
                    return FAILURE;

                pedestrian_position_grid[coordinates.lin][coordinates.col] = pedestrian_set.list[pedestrian_set.num_pedestrians - 1]->id;
            }
            environment_only_grid[coordinates.lin][coordinates.col] = EMPTY_CELL;

            break;
        case '\n':
            break;
        default:
            fprintf(stderr,"Unknow symbol in the environment file: %c.\n", read_char);
            return FAILURE;
    }

    return SUCCESS;
}