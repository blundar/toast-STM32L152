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

		Thanks http://www.x-firm.com/?page_id=193
*/
#include <stdio.h>

#include "ch.h"
#include "hal.h"

#include "pid.h"

#include <math.h>
#include <limits.h>
#include <float.h>

/*
 * Initialize the PID structure.
 * The K values are just 1, a value should not be used in real operation.
 */
void initPIDStruct(PIDStruct *pid)
{
	pid->targetPos=0;
	pid->Direction=PID_FORWARD;
	pid->K=1;
	pid->Kp=1;
	pid->Ki=0;
	pid->Kd=0;
	pid->lastError=0;
	pid->integratedError=0;
	pid->integratedErrorMin = FLT_MIN;
	pid->integratedErrorMax = FLT_MAX;
	pid->lastTicks=0;
}

void resetPIDStruct(PIDStruct *pid, unsigned int ticks_now)
{
	pid->integratedError=0; /* reset integrated error */
	pid->lastError=0;			/* reset last error */
	pid->lastTicks = ticks_now;	/* reset clock to now */
}


/*
 * Get new PID results
 * The resolution of this PID controller is on tick.
 * @Arguments
 * pid							PID controller structure
 * targetPosition		Position we want to go to
 * currentPosition	Position we are currently at
 * ticks						Number of ticks (milliseconds)

 */
uint16_t PIDUpdatePid(PIDStruct *pid, float currentPosition, int ticks)   {
	float error=0;			/* Current Error */
	uint16_t tickDiff=0;		/* Time difference in milliseconds */
	float pTerm=0.0;	/* Proportional term */
	float iTerm=0.0;	/* Integral term */
	float dTerm=0.0;	/* Differential term */
	float Kp, Ki, Kd; /* local coefficients */
	float PIDout;	/* output value as a float */
	uint16_t outVal=0;	/* Output value for PWM */

//Normalize Kp, Ki, Kd for process direction
  if(pid->Direction==PID_REVERSE)
   {
      Kp = (0 - pid->Kp);
      Ki = (0 - pid->Ki);
      Kd = (0 - pid->Kd);
   }
  else {
	  Kp = pid->Kp;
	  Ki = pid->Ki;
	  Kd = pid->Kd;
  }

//Calculate error
	error = pid->targetPos - currentPosition;

	/*
	 * PID code.
	 */
	tickDiff = ticks - pid->lastTicks; /* Time difference in milliseconds */

	/* Proportional term */
	pTerm = Kp * error;
	
	/* Integral term */
	pid->integratedError += error * tickDiff;
	GENconstrain(pid->integratedError, pid->integratedErrorMin, pid->integratedErrorMax);
	iTerm = Ki * pid->integratedError;
	
	/* Differential term */
	dTerm = Kd * (error - pid->lastError) / tickDiff; /* Differential between last en current error, divided by time */

	/* Setup next frame.  */
	pid->lastError = error;
	pid->lastPos = currentPosition;
	pid->lastTicks = ticks;

	/* Calculate output value */
	PIDout = -pid->K*(pTerm + iTerm + dTerm);
	GENconstrain(PIDout, 0, 65535);
	outVal = PIDout;  //convert to unsigned int

	//DEBUG
//	  char buffy[255];
//	  sprintf(buffy,"E %f pT %f iT %f dT %f\r\n", error, pTerm, iTerm, dTerm);
//	  chprintf((BaseChannel *)&SD1, "%s",buffy);

	  outVal = PIDout;  //convert to unsigne int
	return outVal;
}
