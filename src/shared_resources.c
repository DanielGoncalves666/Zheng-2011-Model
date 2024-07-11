/* 
   File: global_declarations.c
   Author: Daniel Gonçalves
   Date: 2023-10-15
   Description: This module contains declarations of enums, structures, constants, and functions that are used throughout the program.
*/

#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>

#include"../headers/grid.h"
#include"../headers/cli_processing.h"
#include"../headers/shared_resources.h"

/**
 * Verifies if the environment_origin selected uses data extracted from an auxiliary file.
 * 
 * @return bool, where True indicates that auxiliary data is used and False otherwise.
*/
bool origin_uses_auxiliary_data()
{
    return cli_args.environment_origin == ONLY_STRUCTURE || 
           cli_args.environment_origin == STRUCTURE_AND_PEDESTRIANS || 
           cli_args.environment_origin == AUTOMATIC_CREATED;
}

/**
 * Verifies if the environment_origin selected uses pedestrians loaded directly from the env-file instead of randomly inserting them.
 * 
 * @return bool, where True indicates that the origin uses static pedestrians is used and False otherwise.
*/
bool origin_uses_static_pedestrians()
{
    return cli_args.environment_origin == STRUCTURE_AND_PEDESTRIANS || 
           cli_args.environment_origin == STRUCTURE_DOORS_AND_PEDESTRIANS;
}

/**
 * Verifies if the environment_origin selected uses exits loaded directly from the env-file instead of inserted them with data from an auxiliary file.
 * 
 * @return bool, where True indicates that the origin uses static exits is used and False otherwise.
*/
bool origin_uses_static_exits()
{
    return cli_args.environment_origin == STRUCTURE_AND_DOORS || 
           cli_args.environment_origin == STRUCTURE_DOORS_AND_PEDESTRIANS;
}