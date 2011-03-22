#ifndef	_ACCELERATION_H
#define	_ACCELERATION_H

/** \file
	\brief interface prototype for acceleration algorithms
*/

#include	"dda_queue.h"

/// pre-calculate as much as possible for this accelerator
/// \param move pointer to move in movebuffer
void acceleration_create(uint8_t move);

/// we're starting up this accelerator
/// \param move pointer to move in movebuffer
/// \return initial step time in cpu ticks
uint32_t acceleration_start(uint8_t move);

/// our timer fired, work out when next to step
/// \param move pointer to move in movebuffer
/// \return time until next step in cpu ticks
uint32_t acceleration_step(uint8_t move);

#endif	/* _ACCELERATION_H */
