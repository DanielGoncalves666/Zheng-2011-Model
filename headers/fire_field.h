#ifndef FIRE_FIELD_H
#define FIRE_FIELD_H

#include"shared_resources.h"
#include"grid.h"

#define NON_RISKY_CELLS 0 // Cells that aren't too close to the fire (since the fire itself isn't considered by any other option, also includes it).
#define RISKY_CELL 1 // A cell close to the fire, but the pedestrians seem it as a calculated risky.
#define DANGER_CELL 2 // A cell too close to the fire. The pedestrians will avoid these cells altogether.

void calculate_fire_floor_field();
void determine_risky_cells();

extern Double_Grid fire_distance_grid;

#endif