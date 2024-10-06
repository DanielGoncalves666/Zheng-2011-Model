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
    enum Simulation_Type simulation_type;
    bool write_to_file;
    bool show_debug_information;
    bool show_simulation_set_info;
    bool immediate_exit;
    bool prevent_corner_crossing;
    bool single_exit_flag;
    bool use_density; // Indicates if the number os pedestrians to be inserted (if the case) is to be based on the density or in the total_num_pedestrians.
    bool fire_is_present;
    int global_line_number;
    int global_column_number;
    int num_simulations;
    int total_num_pedestrians;
    int seed;
    double diagonal;
    double alpha;
    double fire_alpha;
    double fire_gamma;
    double delta;
    double omega;
    double mu;
    double risk_distance;
    double ks;
    double kd;
    double kf;
    double density;
    double min;
    double max;
    double step;
    double spread_rate;
} Command_Line_Args;

error_t parser_function(int key, char *arg, struct argp_state *state);
void extract_full_command(char *full_command, int key, char *arg);
double *obtain_varying_constant();

extern Command_Line_Args cli_args; // cli stands for command line interface
extern const char * argp_program_version;
extern const char doc[];
extern struct argp argp;

#endif