/* 
   File: global_declarations.c
   Author: Daniel Gonçalves
   Date: 2023-10-15
   Description: This module contains declarations of enums, structures, constants, and functions that are used throughout the program.
*/

#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>
#include<math.h>

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
 * @return bool, where True indicates that the origin uses static pedestrians and False otherwise.
*/
bool origin_uses_static_pedestrians()
{
    return cli_args.environment_origin == STRUCTURE_AND_PEDESTRIANS || 
           cli_args.environment_origin == STRUCTURE_DOORS_AND_PEDESTRIANS;
}

/**
 * Verifies if the selected environment_origin uses exits loaded directly from the env-file instead of inserting them with data from an auxiliary file.
 * 
 * @return bool, where True indicates that the origin uses static exits and False otherwise.
*/
bool origin_uses_static_exits()
{
    return cli_args.environment_origin == STRUCTURE_AND_DOORS || 
           cli_args.environment_origin == STRUCTURE_DOORS_AND_PEDESTRIANS;
}

/**
 * Calculates the Euclidean distance between the provided coordinates.
 * @param first The first pair of coordinates.
 * @param second The second pair of coordinates.
 * 
 * @return A double, the euclidean distance between the two pair of coordinates provided.
 */
double euclidean_distance(Location first, Location second)
{
    return sqrt(pow(first.lin - second.lin, 2) + pow(first.col - second.col, 2));
}

/**
 * Generates a random floating-point number within a specified range [min, max].
 * 
 * @param min The lower bound of the range (inclusive).
 * @param max The upper bound of the range (inclusive).
 * 
 * @return A random floating-point number between `min` and `max`, inclusive.
 */
float rand_within_limits(float min, float max) 
{
    float range = (max - min); 
    double div = RAND_MAX / range;
    return min + (rand() / div);
}

/**
 * Determines whether a event occurs based on a probability.
 * 
 * @param probability The probability that an event will occur. Must be a value between 0 and 1 (inclusive).
 * 
 * @return True, if the draw number is less than the given probability (i.e., the event happens), or False, if otherwise.
 */
bool probability_test(double probability)
{
    float draw_number = rand_within_limits(0,1);

    return draw_number < probability;
}