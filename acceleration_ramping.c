/// whether we accelerate, run at full speed, break down, etc.
typedef enum {
	RAMP_UP,
	RAMP_MAX,
	RAMP_DOWN
} ramp_state_t;

/// this struct holds all the ramping-specific data for each move
typedef struct {
	// bresenham counters
	int32_t						x_counter; ///< counter for total_steps vs this axis, used for bresenham calculations.
	int32_t						y_counter; ///< counter for total_steps vs this axis, used for bresenham calculations.
	int32_t						z_counter; ///< counter for total_steps vs this axis, used for bresenham calculations.
	int32_t						e_counter; ///< counter for total_steps vs this axis, used for bresenham calculations.

	/// start of down-ramp, intitalized with total_steps / 2
	uint32_t					ramp_steps;
	/// counts actual steps done
	uint32_t					step_no;
	/// 24.8 fixed point timer value, maximum speed
	uint32_t					c_min;
	/// tracking variable
	int32_t						n;
	/// keep track of whether we're ramping up, down, or plateauing
	ramp_state_t			ramp_state;
} ACCEL_DATA_RAMPING;

/// ramping data buffer
ACCEL_DATA_RAMPING accel_data_buffer[MOVEBUFFER_SIZE];

// ***************************************************

void acceleration_create(uint8_t move) {
	uint32_t distance;
	DDA *dda = &movebuffer[move];
	ACCEL_DATA_RAMPING *aad = &accel_data_buffer[move];

	dda->x_counter = dda->y_counter = dda->z_counter = dda->e_counter = -(dda->total_steps >> 1);

	// since it's unusual to combine X, Y and Z changes in a single move on reprap, check if we can use simpler approximations before trying the full 3d approximation.
	if (dda->z_delta == 0)
		distance = approx_distance(dda->x_delta * UM_PER_STEP_X, dda->y_delta * UM_PER_STEP_Y);
	else if (dda->x_delta == 0 && dda->y_delta == 0)
		distance = dda->z_delta * UM_PER_STEP_Z;
	else
		distance = approx_distance_3(dda->x_delta * UM_PER_STEP_X, dda->y_delta * UM_PER_STEP_Y, dda->z_delta * UM_PER_STEP_Z);

	// if we're just extruding, distance is simply E
	if (distance < 2)
		distance = dda->e_delta * UM_PER_STEP_E;

	// pre-calculate move speed in millimeter microseconds per step minute for less math in interrupt context
	// mm (distance) * 60000000 us/min / step (total_steps) = mm.us per step.min
	//   note: um (distance) * 60000 == mm * 60000000
	// so in the interrupt we must simply calculate
	// mm.us per step.min / mm per min (F) = us per step

	// break this calculation up a bit and lose some precision because 300,000um * 60000 is too big for a uint32
	// calculate this with a uint64 if you need the precision, but it'll take longer so routines with lots of
	//   short moves may suffer
	// 2^32/6000 is about 715mm which should be plenty

	// changed * 10 to * (F_CPU / 100000) so we can work in cpu_ticks rather than microseconds.
	// timer.c setTimer() routine altered for same reason

	// changed distance * 6000 .. * F_CPU / 100000 to
	//         distance * 2400 .. * F_CPU / 40000 so we can move a distance of up to 1800mm without overflowing
	uint32_t mmticks-per-stepmin = ((distance * 2400) / dda->total_steps) * (F_CPU / 40000);

	aad->c = (mmticks-per-stepmin / startpoint.F) << 8;

	// add the last bit of dda->total_steps to always round up
	aad->ramp_steps = dda->total_steps / 2 + (dda->total_steps & 1);
	aad->step_no = 0;
	// c is initial step time in IOclk ticks
	aad->c = ACCELERATION_STEEPNESS << 8;
	aad->c_min = (move_duration / target->F) << 8;
	if (aad->c_min < c_limit)
		aad->c_min = c_limit;
	aad->n = 1;
	aad->ramp_state = RAMP_UP;

	aad->x_counter = aad->y_counter = aad->z_counter = aad->e_counter = 0;
}

uint32_t acceleration_start(uint8_t move) {
	// simply return precalculated step timing
	return movebuffer[move].c;
}

uint32_t acceleration_step(uint8_t move) {
	// - algorithm courtesy of http://www.embedded.com/columns/technicalinsights/56800129?printable=true
	// - for simplicity, taking even/uneven number of steps into account dropped
	// - number of steps moved is always accurate, speed might be one step off
	switch (aad->ramp_state) {
		case RAMP_UP:
		case RAMP_MAX:
			if (aad->step_no >= aad->ramp_steps) {
				// RAMP_UP: time to decelerate before reaching maximum speed
				// RAMP_MAX: time to decelerate
				aad->ramp_state = RAMP_DOWN;
				aad->n = -((int32_t)2) - aad->n;
			}
			if (aad->ramp_state == RAMP_MAX)
				break;
		case RAMP_DOWN:
			aad->n += 4;
			// be careful of signedness!
			aad->c = (int32_t)aad->c - ((int32_t)(aad->c * 2) / aad->n);
			if (aad->c <= aad->c_min) {
				// maximum speed reached
				aad->c = aad->c_min;
				aad->ramp_state = RAMP_MAX;
				aad->ramp_steps = dda->total_steps - dda->step_no;
			}
			break;
	}
	aad->step_no++;

	return aad->c;
}
