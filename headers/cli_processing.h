#ifndef CLI_PROCESSING_H
#define CLI_PROCESSING_H

#include<argp.h>
#include<stdbool.h>

#include"shared_resources.h"

typedef struct{
    char full_command[600];
    char environment_filename[150];
    char output_filename[150];
    char auxiliary_filename[150];
    enum Output_Format output_format;
    enum Environment_Origin environment_origin;
    bool write_to_file;
    bool show_debug_information;
    bool show_simulation_set_info;
    bool immediate_exit;
    bool always_move_to_lowest;
    bool prevent_corner_crossing;
    bool allow_X_movement;
    bool single_exit_flag;
    int global_line_number;
    int global_column_number;
    int num_simulations;
    int total_num_pedestrians;
    int seed;
    double alpha;
    double diagonal;
} Command_Line_Args;

error_t parser_function(int key, char *arg, struct argp_state *state);
void extract_full_command(char *full_command, int key, char *arg);

extern Command_Line_Args cli_args; // cli stands for command line interface
extern const char * argp_program_version;
extern const char doc[];
extern struct argp argp;

#endif