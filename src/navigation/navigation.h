#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "kalman/ukf_mrp.h"
#include "hardware/flight-input.h"

typedef drone_state {

	Ukf_parameters ukf_param;

} Drone_state;



#endif
