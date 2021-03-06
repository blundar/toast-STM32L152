/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

//Enums for direct / reverse control
#define PID_FORWARD 0
#define PID_REVERSE 1

/*  Structure holding the data of the PID-controller instances. */
typedef struct {
	float targetPos;	/* Position to control value to */
	int Direction;		/* forward or reverse feedback */
	float K;				/* Overall gain */
	float Kp;			/* Proportional gain */
	float Ki;			/* Integral gain */
	float Kd;			/* Differential gain */
	float lastError;	/* Last error to feed to the differential function */
	float lastPos;		/* last position */
	float integratedError; /* Integrated Error */
	float integratedErrorMin;	/* Constraint for integrated Error */
	float integratedErrorMax;	/* Constraint for integrated Error */
	unsigned int		lastTicks;	/* prior time now */
} PIDStruct;

/*
 * Get new PID results
 * The resolution of this PID controller is on tick.
 */
uint16_t PIDUpdatePid(PIDStruct *pid, float currentPosition, int ticks);

/*
 * Initialize the PID structure.
 * The K values are just 1, a value should not be used in real operation.
 */
void initPIDStruct(PIDStruct *pid);

/*
 * constrain values to range.
 */
#define GENconstrain(v, vmin, vmax){																					\
	if (v <= vmin)																															\
		{ v = vmin; }																															\
	else if (v >= vmax)																													\
		{ v = vmax; }																															\
}
