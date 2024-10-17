#ifndef FIRE_DYNAMICS_H
#define FIRE_DYNAMICS_H

#include"shared_resources.h"
#include"grid.h"

#define NON_RISKY_CELLS -2000 // A non risky cell is a cell too far away from a fire.
#define RISKY_CELL -2001 // A risky cell is a cell between a fire and a wall.
#define DANGER_CELL -2002 // A danger cell is a cell adjacent to a fire that isn't a risky cell.
#define FIRE_CELL -2003 // A cell with fire

void zheng_fire_propagation();
void determine_risky_cells();
void calculate_fire_floor_field();

extern Double_Grid fire_distance_grid;
extern Int_Grid fire_grid;
extern Int_Grid initial_fire_grid;
extern Int_Grid risky_cells_grid;

#endif