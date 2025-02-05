#ifndef DYNAMIC_FIELD_H
#define DYNAMIC_FIELD_H

#include"shared_resources.h"

void increase_particle_at(Location coordinates);
void increase_particle_fast_only_trace(Location coordinates);
Function_Status apply_decay_and_diffusion();
Function_Status alternative_apply_decay_and_diffusion();

#endif