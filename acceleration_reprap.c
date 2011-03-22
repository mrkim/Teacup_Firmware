void acceleration_create(uint8_t move) {
	uint32_t distance;
	DDA *dda = &movebuffer[move];

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
	uint32_t mmticks-per-stepmin = ((distance * 2400) / movebuffer[move].total_steps) * (F_CPU / 40000);

	movebuffer[move].c = (mmticks-per-stepmin / startpoint.F) << 8;
}

uint32_t acceleration_start(uint8_t move) {
	// simply return precalculated step timing
	return movebuffer[move].c;
}

uint32_t acceleration_step(uint8_t move) {
	// simply return precalculated step timing
	return movebuffer[move].c;
}
