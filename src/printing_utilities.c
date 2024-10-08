/* 
   File: printing_utilities.c
   Author: Daniel Gonçalves
   Date: 2023-10-15
   Description: This module contains functions to print some of the data generated by the simulations.
*/

#include<stdio.h>
#include<string.h>
#include<time.h>

#include"../headers/exit.h"
#include"../headers/fire_dynamics.h"
#include"../headers/pedestrian.h"
#include"../headers/cli_processing.h"
#include"../headers/printing_utilities.h"
#include"../headers/shared_resources.h"

/**
 * Print the command received by CLI on the provided stream.
 * 
 * @param output_stream Stream where the data will be written.
*/
void print_full_command(FILE *output_stream)
{
	if(output_stream != NULL)
	{
		fprintf(output_stream, "./zheng.sh%s", cli_args.full_command);
		fprintf(output_stream,"\n--------------------------------------------------------------\n\n");
	}
	else
		fprintf(stderr, "No valid stream was provided at print_full_command.\n");
}

/**
 * Print the heatmap grid on the provided stream.
 * 
 * @note The value of each position of the grid is divided by the number of simulations in order to achieve the mean of all simulations.
 * 
 * @param output_stream Stream where the data will be written.
*/
void print_heatmap(FILE *output_stream)
{
	if(output_stream != NULL)
	{
		for(int i = 0; i < cli_args.global_line_number; i++){
			for(int j = 0; j < cli_args.global_column_number; j++)
				fprintf(output_stream, "%.2lf ", (double) heatmap_grid[i][j] / (double) cli_args.num_simulations);

			fprintf(output_stream,"\n");
		}
		fprintf(output_stream,"\n");
	}
	else
		fprintf(stderr, "No valid stream was provided at print_heatmap.\n");
}

/**
 * Prints all elements of the environment (pedestrians, obstacles, walls, fire, exits and empty cells) to the provided stream.
 * 
 * @param output_stream Stream where the data will be written.
 * @param simulation_number Current simulation index
 * @param timestep Current simulation timestep.
*/
void print_complete_environment(FILE *output_stream, int simulation_number, int timestep)
{
	if(!cli_args.write_to_file)
		printf("\e[1;1H\e[2J");

	fprintf(output_stream,"Simulation %d - timestep %d\n\n",simulation_number, timestep);

	if(output_stream != NULL)
	{
		for(int i = 0; i < cli_args.global_line_number; i++){
			for(int j = 0; j < cli_args.global_column_number; j++)
			{
				if(pedestrian_position_grid[i][j] != 0)
				{
					if(fire_grid[i][j] == FIRE_CELL)
						fprintf(output_stream, "🪦");
					else
						fprintf(output_stream,"👤");
				}
				else if(fire_grid[i][j] == FIRE_CELL)
					fprintf(output_stream, "🔥");
				else if(exits_only_grid[i][j] == EXIT_CELL)
					fprintf(output_stream,"🚪");
				else if(obstacle_grid[i][j] == IMPASSABLE_OBJECT)
					fprintf(output_stream,"🧱");
				else if(pedestrian_position_grid[i][j] == 0)
					fprintf(output_stream,"⬛");
			}
			fprintf(output_stream,"\n");
		}
		fprintf(output_stream,"\n");
	}
	else
		fprintf(stderr, "No valid stream was provided at print_complete_environment.\n");		
}

/**
 * Prints the int_grid to the specified stream.
 * 
 * @param int_grid Integer grid to be printed.
*/
void print_int_grid(FILE *output_stream, Int_Grid int_grid)
{
	for(int i = 0; i < cli_args.global_line_number; i++){
		for(int j = 0; j < cli_args.global_column_number; j++){
			fprintf(output_stream, "%3d ", int_grid[i][j]);
		}
		printf("\n\n");
	}
	printf("\n");
}

/**
 * Prints the double_grid (using the provided precision) to the specified stream.
 * 
 * @param output_stream Stream where the data will be written.
 * @param double_grid Double_Grid to be printed.
 * @param precision Precision of the printed value.
 * 
*/
void print_double_grid(FILE *output_stream, Double_Grid double_grid, int precision)
{
	if(precision < 0)
		precision = 0;

	for(int i = 0; i < cli_args.global_line_number; i++){
		for(int j = 0; j < cli_args.global_column_number; j++)

			if(double_grid[i][j] < 0)
				fprintf(output_stream, "%*.0f ", 2 + precision, double_grid[i][j]);
			else
				fprintf(output_stream, "%5.*lf ", precision, double_grid[i][j]);

		printf("\n\n");
	}
	printf("\n");
}

/**
 * Prints the double_grid multiplied by the given value (using the provided precision) to the specified stream.
 * 
 * @note The multiplication doesn't alter the values within the grid. The operation is external to them.
 * 
 * @param output_stream Stream where the data will be written.
 * @param double_grid Double_Grid to be printed.
 * @param precision Precision of the printed value.
 * @param value The value that will be multiplied to the grid.
 * 
*/
void multiply_and_print_double_grid(FILE *output_stream, Double_Grid double_grid, int precision, double value)
{
	if(precision < 0)
		precision = 0;

	for(int i = 0; i < cli_args.global_line_number; i++){
		for(int j = 0; j < cli_args.global_column_number; j++)

			if(double_grid[i][j] < 0)
				fprintf(output_stream, "%*.0f ", 2 + precision, value * double_grid[i][j]);
			else
				fprintf(output_stream, "%5.*lf ", precision, value * double_grid[i][j]);

		printf("\n\n");
	}
	printf("\n");
}


/**
 * Print information about the exits of a simulation set.
 * 
 * @param output_stream Stream where the data will be written.
*/
void print_simulation_set_information(FILE *output_stream)
{
	char separator = ',';
    char aggregator = '+';

	if(output_stream != NULL)
	{
		fprintf(output_stream, "Simulation set:");
		for(int exit_index = 0; exit_index < exits_set.num_exits; exit_index++)
		{
			if(exit_index == exits_set.num_exits - 1)
				separator = '.';

			Exit current_exit = exits_set.list[exit_index];
			
			int exit_width = current_exit->width;
			for(int cell_index = 0; cell_index < exit_width; cell_index++)
			{ 
				Location cell = current_exit->coordinates[cell_index];
				fprintf(output_stream, " %d %d%c", cell.lin, cell.col, 
										cell_index == exit_width - 1 ? separator : aggregator);
			}

		}

		fprintf(output_stream, "\n");
	}
	else
		fprintf(stderr, "No valid stream was provided at print_simulation_set_information.\n");
}

/**
 * Print a status message about the execution of the program to stdout.
 * 
 * @param set_index Current simulation set index.
 * @param set_quantity The number of simulations sets.
*/
void print_execution_status(int set_index, int set_quantity)
{
	char date_time[51];
            
	time_t current_time = time(NULL);
	struct tm * time_information = localtime(&current_time);
	
	if(set_index != 0)
	{
		fprintf(stdout, "\033[A\033[2K");
		/*
			\033: Represents the escape character, equivalent to ESC.
		  	[A: Moves the cursor up one line.
		  	[2K: Clears the entire current line in the terminal.
		 */
		fflush(stdout);
	}
	
	strftime(date_time,50,"%F %Z %T",time_information);
	fprintf(stdout, "Simulation set %5d/%d finalized at %s.\n", set_index + 1, set_quantity, date_time);
}

/**
 * Prints the given value `cli_args.num_simulation` times to the provided `stream`. The printed values serve as placeholders for simulations with invalid parameters, such as inaccessible exits.
 * 
 * @param stream Stream where the data will be written.
 * @param placeholder Value that will be printed.
*/
void print_placeholder(FILE *stream, int placeholder)
{
	for(int times = 0; times < cli_args.num_simulations; times++)
	{
		fprintf(stream, "%d ", placeholder);
	}
	fprintf(stream, "\n");
}
