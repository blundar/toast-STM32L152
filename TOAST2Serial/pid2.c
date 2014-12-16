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

#include "ch.h"
#include "hal.h"

#include "pid.h"

#include <math.h>
#include <limits.h>

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
	pid->integratedErrorMin = INT_MIN;
	pid->integratedErrorMax = INT_MAX;
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
 * @Returns
 * correction (-256 to 256)
 */
uint16_t PIDUpdatePid(PIDStruct *pid, float currentPosition, int ticks)   {
	float error=0;			/* Current Error */
	uint16_t tickDiff=0;		/* Time difference in milliseconds */
	float pTerm=0.0;	/* Proportional term */
	float iTerm=0.0;	/* Integral term */
	float dTerm=0.0;	/* Differential term */
	float Kp, Ki, Kd; /* local coefficients */
	uint16_t outVal=0;	/* Output value for PWM */

//Normalize Kp, Ki, Kd for process direction
  if(pid->Direction==PID_REVERSE)
   {
      Kp = (0 - pid->Kp);
      Ki = (0 - pid->Ki);
      Kd = (0 - pid->Kd);
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
	dTerm = pid->Kd * (error - pid->lastError) / tickDiff; /* Differential between last en current error, divided by time */

	/* Setup next frame.  */
	pid->lastError = error;
	pid->lastPos = currentPosition;
	pid->lastTicks = ticks;

	/* Calculate output value */
	outVal = -pid->K*(pTerm + iTerm + dTerm);
	GENconstrain(outVal, 0, 65535);
	return outVal;
}

/*working variables*/
unsigned long lastTime;
double Input, Output, Setpoint;
double ITerm, lastInput;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
double outMin, outMax;
bool inAuto = false;
 
#define MANUAL 0
#define AUTOMATIC 1
 
#define PID_DIRECT 0
#define PID_REVERSE 1
int controllerDirection = DIRECT;
 
void Compute()
{
//   if(!inAuto) return;
//   unsigned long now = millis();
//   int timeChange = (now - lastTime);
//   if(timeChange>=SampleTime)
//   {
//		Assume that we're called at regular intervals.
		
      /*Compute all the working error variables*/
      double error = Setpoint - Input;
      
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      Output = kp * error + ITerm- kd * dInput;
      if(Output > outMax) Output = outMax;
      else if(Output < outMin) Output = outMin;
 
      /*Remember some variables for next time*/
      lastInput = Input;
      lastTime = now;
//  }
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
   if (Kp<0 || Ki<0|| Kd<0) return;
 
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec;
 
  if(controllerDirection ==REVERSE)
   {
      kp = (0 - kp);
      ki = (0 - ki);
      kd = (0 - kd);
   }
}
 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime
                      / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
 
void SetOutputLimits(double Min, double Max)
{
   if(Min > Max) return;
   outMin = Min;
   outMax = Max;
 
   if(Output > outMax) Output = outMax;
   else if(Output < outMin) Output = outMin;
 
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}
 
void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}
 
void Initialize()
{
   lastInput = Input;
   ITerm = Output;
   if(ITerm > outMax) ITerm= outMax;
   else if(ITerm < outMin) ITerm= outMin;
}
 
void SetControllerDirection(int Direction)
{
   controllerDirection = Direction;
}