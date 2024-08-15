/* 
   File: pedestrian.c
   Author: Daniel Gonçalves
   Date: 2024-06-20
   Description: This module defines a structure related to a single cell and several functions that operate with it. These functions include finding the smallest neighbor of a cell, sorting a list of cell structures, and counting the number of cells with a value smaller than a specified cell.
*/

#include<stdlib.h>
#include<stdbool.h>

#include"../headers/cell.h"
#include"../headers/exit.h"
#include"../headers/grid.h"
#include"../headers/shared_resources.h"

static void insertion_sort(Cell *cell_list, int start_index, int end_index);
static int count_cells_equal(Cell *cell_list, int list_length, int value_index, int *smaller_index);
static int partition(Cell *cell_vector, int start_index, int end_index);
static void swap(Cell *cell_list, int a, int b);

/**
 * Perform a binary search for the given value in the provided list of Cell structures and determine the number of cells with values smaller than of the searched value. In additional, count the number of cells equal to the searched value.
 * 
 * @note cell_list must be ordered.
 *  
 * @param cell_list  Pointer to the array of Cell structures to be searched.
 * @param list_length Number of elements in the given list of Cells.
 * @param searched_value Value to be searched.
 * @param equal_quantity A pointer to where the number of cells equal to the searched_value will be stored.
 * 
 * @return An integer, where -1 indicates that the searched value wasn't found or that the provided cell list isn't valid, and a non-negative value indicates the number of cells with values smaller than the searched value.
*/
int count_cells_with_smaller_value(Cell *cell_list, int list_length, double searched_value, int *equal_quantity)
{
    if(cell_list == NULL)
        return -1;

    int left = 0;
    int right = list_length - 1;
    int middle;

    int smaller_quantity = -1;
    *equal_quantity = 0;

    if(searched_value < cell_list[0].value)
        return -1;

    if(searched_value > cell_list[list_length - 1].value)
        return list_length;

    while(left <= right)
    {
        middle = (left + right) / 2;

        if(cell_list[middle].value == searched_value)
        {
            *equal_quantity = count_cells_equal(cell_list, list_length, middle, &smaller_quantity);

            break;
        }

        if(cell_list[middle].value > searched_value)
            right = middle - 1;
        else if(cell_list[middle].value < searched_value) 
            left = middle + 1;
    }

    if(smaller_quantity == -1) // No cell with searched_value as value.
    {
        // By the binary search algorithm, all cells with index less than the current value of `left` are smaller than the searched value.
        // Therefore, the number of cells smaller than the searched value is exact `left` (from indexes 0 to left -1).

        smaller_quantity = left;

        // The cell at the left index can be smaller than the searched value, so is necessary to check it.
        if(cell_list[left].value < searched_value)
            smaller_quantity++;
    }

    return smaller_quantity;
}

/**
 * Sorts a list of Cell structures in ascending order with the quicksort algorithm.
 * 
 * @param cell_list  Pointer to the array of Cell structures to be sorted.
 * @param start_index The starting index of the interval to be sorted.
 * @param end_index The ending index (inclusive) of the interval to be sorted.
*/
void quick_sort(Cell *cell_list, int start_index, int end_index)
{
    if(end_index - start_index + 1 < 10)
    {
        insertion_sort(cell_list, start_index, end_index);    
        return;
    }

    int pivot = partition(cell_list, start_index, end_index);
    quick_sort(cell_list, start_index, pivot - 1);
    quick_sort(cell_list, pivot + 1, end_index);
}


/* ---------------- */
/* STATIC FUNCTIONS */
/* ---------------- */

/**
 * Sorts the given Cell list in ascending order with the insertion sort algorithm.
 *  
 * @param cell_list A Cell list to be sorted.
 * @param start_index The starting index of the interval to be sorted.
 * @param end_index The ending index (inclusive) of the interval to be sorted.
*/
static void insertion_sort(Cell *cell_list, int start_index, int end_index)
{
    if(start_index < 0 || end_index < 0 || end_index - start_index < 0)
        return; // Invalid Interval

    for(int i = start_index + 1; i <= end_index; i++)
    {
        Cell current = cell_list[i];

        int h = i - 1;
        // shift elements to the right
        while(h >= start_index && current.value < cell_list[h].value)
        {
            cell_list[h + 1] = cell_list[h];
            h--;
        }
        
        cell_list[h + 1] = current;
    }
}

/**
 * Search the nearby cells of the given index (in both directions) and count the number of cells with the same value. 
 * 
 * @note cell_list must be ordered.
 * 
 * @param cell_list Pointer to an array of Cell structures.
 * @param list_length Number of elements in the given list of Cells.
 * @param searched_value Index of the cell in the array of Cells which value will be counted.
 * @param smaller_index A pointer to an integer where the smaller index for a cell with the same value as cell_list[value_index] will be stored.
 * 
 * @return An integer, indicating the number of cells with the same value of cell_list[value_index].
*/
static int count_cells_equal(Cell *cell_list, int list_length, int value_index, int *smaller_index)
{
    if(cell_list == NULL)
        return 0;

    double searched_value = cell_list[value_index].value;
    int number_equals = 0;

    int index = value_index;

    // Verify for equal values to the left.
    while(index >= 0 && cell_list[index].value == searched_value)
    {
        index--;
        number_equals++;
    }
    *smaller_index = index + 1; // Smaller index where exits a cell with the same value as cell_list[value_index].
                                // This is the same as the number of cells with a value less than cell_list[value_index].

    index = value_index + 1;

    // Verify for equal values to the right
    while(index < list_length && cell_list[index].value == searched_value)
    {
        index++;
        number_equals++;
    }

    return number_equals;
}

/**
 * Partition function of the quicksort algorithm. Determines the pivot and partitions the array such that elements
 * lower than the pivot are on the left side, the pivot itself is in the middle, and elements higher than the
 * pivot are on the right side.
 * 
 * @param cell_vector  Pointer to the array of Cell structures to be sorted.
 * @param start_index The starting index of the interval to be sorted.
 * @param end_index The ending index (inclusive) of the interval to be sorted.
 * 
 * @return Index of the pivot element after partitioning.
*/
static int partition(Cell *cell_vector, int start_index, int end_index)
{
    int pivot = start_index;
    for(int i = pivot + 1; i <= end_index; i++)
    {
        if(cell_vector[i].value < cell_vector[start_index].value)
        {
            pivot = pivot + 1;
            swap(cell_vector,pivot,i);
        }
    }
    swap(cell_vector,start_index,pivot);
    return pivot;
}

/** 
 * Swap the cells at the given positions. 
 * 
 * @param cell_vector  Pointer to the array of Cell structures where the positions will be swapped.
 * @param first_index Index of the first Cell.
 * @param second_index Index of the second Cell.
*/
static void swap(Cell *cell_vector, int first_index, int second_index)
{
    Cell buffer = cell_vector[first_index];
    cell_vector[first_index] = cell_vector[second_index];
    cell_vector[second_index] = buffer;
}
