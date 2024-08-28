#ifndef DYNAMIC_FIELD_H
#define DYNAMIC_FIELD_H

#include"shared_resources.h"

void increase_particle_at(Location coordinates);
void decay();
Function_Status single_diffusion(bool is_moving);
Function_Status multiple_diffusion();

#endif