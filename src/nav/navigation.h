#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "../kalman/ukf_mrp.h"
#include "../hardware/flight-input.h"
#include "../pid/pid.h"

typedef struct drone_state {

	int motors[4];
	Pidhist hist_x;
	Pidhist hist_y;
	Ukf_parameters ukf_param;

} Drone_state;

int init_nav(Drone_state *state);
// start nav system not including alignment, initialize state
// returns 0 on success, non-zero value indicates failure

int align_nav(Drone_state *state);
// align state with data from sensors
// returns 0 on success, non-zero value indicates failure

int update_nav(Drone_state *state, Controls *controls, double delta_t);
// read from sensors, perform single update of kalman filter and pid
// returns 0 on success, non-zero value indicates failure

void stop_nav();
// shuts down sensors and the sensor and motor threads

#endif
