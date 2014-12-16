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
#include "ch.h"
#include "hal.h"

#include "chprintf.h"

//Grab our temp curve
#include "desired_temps.h"

//PID stuff
#include "pid.h"

//ghetto global
   adcsample_t avg_ch1, avg_ch2;

   uint8_t running, preheat;
   uint16_t fan_duty, heat_duty;
   uint32_t seconds_counter, clk_ticks;

   PIDStruct heatPID;

    
//convert from ADC readings into degrees C
unsigned int adc_ad8495_conv(adcsample_t reading) {
//	int temperature;
	unsigned int temperature_x4;
	
//	temperature = reading * 5 / 31;		//AD8495 = 0.05v/degree C.  To convert raw -> degrees C, divide by 31 + multiply by 5
	temperature_x4 = reading * 20 / 31;		//AD8495 = 0.05v/degree C.  To convert raw -> degrees C, divide by 31 + multiply by 5

	
	return(temperature_x4);
}
    
//static void pwmpcb(PWMDriver *pwmp);
static void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n);

/* Total number of channels to be sampled by a single ADC operation.*/
#define ADC_GRP1_NUM_CHANNELS   2

/* Depth of the conversion buffer, channels are sampled four times each.*/
#define ADC_GRP1_BUF_DEPTH      4

/*
 * ADC samples buffer.
 */
static adcsample_t samples[ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH];

/*
 * ADC conversion group.
 * Mode:        Linear buffer, 4 samples of 2 channels, SW triggered.
 * Channels:    IN10   (48 cycles sample time)
 *              Sensor (192 cycles sample time)
 */
static const ADCConversionGroup adcgrpcfg = {
  FALSE,
  ADC_GRP1_NUM_CHANNELS,
  adccb,
  NULL,
  /* HW dependent part.*/
  0,                        /* CR1 */
  ADC_CR2_SWSTART,          /* CR2 */
  0,
  ADC_SMPR3_SMP_AN5(ADC_SAMPLE_48) | ADC_SMPR2_SMP_SENSOR(ADC_SAMPLE_192),
  0,
  ADC_SQR1_NUM_CH(ADC_GRP1_NUM_CHANNELS),
  0,
  0,
  0,
  ADC_SQR5_SQ2_N(ADC_CHANNEL_IN5) | ADC_SQR5_SQ1_N(ADC_CHANNEL_SENSOR)
};

/*
 * PWM configuration structure.
 * Cyclic callback enabled, channels 1 and 2 enabled without callbacks,
 * the active state is a logic one.
 */
static PWMConfig pwmcfg = {
  10000,                                    /* 10kHz PWM clock frequency.   */
  2500,                                    /* PWM period 1S (in ticks).    */
//  pwmpcb,
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL}
  },
  /* HW dependent part.*/
  0
};

/*
 * PWM cyclic callback.
 * A new ADC conversion is started.
 */
static void adc_go(void) {

  /* Starts an asynchronous ADC conversion operation, the conversion
     will be executed in the background */
  chSysLockFromIsr();
  adcStartConversionI(&ADCD1, &adcgrpcfg, samples, ADC_GRP1_BUF_DEPTH);
  chSysUnlockFromIsr();
}

/*
 * ADC end conversion callback.
 * The PWM channels are reprogrammed using the latest ADC samples.
 * The latest samples are transmitted into a single SPI transaction.
 */
void adccb(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

  (void) buffer; (void) n;
  /* Note, only in the ADC_COMPLETE state because the ADC driver fires an
     intermediate callback when the buffer is half full.*/
  if (adcp->state == ADC_COMPLETE) {
//    adcsample_t avg_ch1, avg_ch2;

    /* Calculates the average values from the ADC samples.*/
    avg_ch1 = (samples[0] + samples[2] + samples[4] + samples[6]) / 4;
    avg_ch2 = (samples[1] + samples[3] + samples[5] + samples[7]) / 4;

    chSysLockFromIsr();

    /* SPI slave selection and transmission start.*/
    spiSelectI(&SPID2);
    spiStartSendI(&SPID2, ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH, samples);

    chSysUnlockFromIsr();
  }
}

/*
 * This is a periodic thread that does absolutely nothing except increasing
 * a seconds counter.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

(void)arg;
  chRegSetThreadName("counter");
  
  while (TRUE) {
    chThdSleepMilliseconds(1000);
    adc_go();
    
    if(running) { 
    	seconds_counter++; 
    	}
  }
}

/*
 * Application entry point.
 */
int main(void) {
	
	//some local variables
	unsigned int temp_x4, temp_degc, target_x4, target_degc;
	signed int error_x4, error_degc;

	uint8_t PWM_active;


	//initialize some variables
	seconds_counter = 0;
	PWM_active = 0;


  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Activates the serial driver 1 using the driver default configuration.
   * PA9 and PA10 are routed to USART1.
   */
  sdStart(&SD1, NULL);
  palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(7));


  /*
   * Initializes the ADC driver 1 and enable the thermal sensor.
   * The pin PA5 on the port GPIOC is programmed as analog input.
   */
  adcStart(&ADCD1, NULL);
  adcSTM32EnableTSVREFE();
  palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);


//initialize variables
	running = FALSE; 	//by default: not running
	preheat = FALSE;
	
//PID
	initPIDStruct(&heatPID);
//	heatPID.K=1.2;	//increase overall gain
	heatPID.Kp=-600.0;  //was 151  v2=-600 v3=500 v4=1000
	heatPID.Ki=-.1; //v2 = -.05 v3= .04 v4=0.1
	heatPID.Kd=-2000000; // was 500000 v2=-5000000 v3=-2000000 v4= -6000000
	heatPID.integratedErrorMin=-300000; // v3 = -400000
	heatPID.integratedErrorMax=300000; // v3 = 300000 v4=150000

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  //Announce ourselves
  chprintf((BaseChannel *)&SD1, "\r\nHello.  This is a Chibios based temperature controller.\r\nTemp input PA5 0.05volt/Deg C.\r\nPB6 heater control PWM 4hz carrier\r\nPB7 fan control PWM 4 hz carrier.\r\n\r\n");

	//Start PWM up if necessary
	if (PWM_active ==0) {
		  /*
		   * Initializes the PWM driver 4, routes the TIM4 outputs to the board LEDs.
		   */
		  pwmStart(&PWMD4, &pwmcfg);
		  palSetPadMode(GPIOB, GPIOB_LED4, PAL_MODE_ALTERNATE(2));
		  palSetPadMode(GPIOB, GPIOB_LED3, PAL_MODE_ALTERNATE(2));
		  pwmEnableChannelI(&PWMD4, 0, PWM_FRACTION_TO_WIDTH(&PWMD4, 2048, 768));
		  pwmEnableChannelI(&PWMD4, 1, PWM_FRACTION_TO_WIDTH(&PWMD4, 2048, 0));
		  PWM_active = 1;	//so we don't re-activate the PWM system
		}


  /*
   * Normal main() thread activity, in this demo it does nothing except
   * sleeping in a loop and check the button state, when the button is
   * pressed the test procedure is launched with output on the serial
   * driver 1.
   */
  while (TRUE) {
    if (palReadPad(GPIOA, GPIOA_BUTTON)) {
    	preheat = TRUE;
    }
    if (preheat == TRUE){
		pwmEnableChannelI(&PWMD4, 0, PWM_FRACTION_TO_WIDTH(&PWMD4, 2048, 512));
	    pwmEnableChannelI(&PWMD4, 1, PWM_FRACTION_TO_WIDTH(&PWMD4, 2048, 2048));
	    temp_x4 = adc_ad8495_conv(avg_ch2);
	    while (temp_x4 <140) {
	    	temp_x4 = adc_ad8495_conv(avg_ch2);
	        temp_degc = temp_x4 / 4;
	        chprintf((BaseChannel *)&SD1, "%d Preheat 100% Temp: %d deg\r\n", seconds_counter, temp_degc);
	        chThdSleepMilliseconds(1000);
	        seconds_counter++;
	    	}
	    pwmEnableChannelI(&PWMD4, 1, PWM_FRACTION_TO_WIDTH(&PWMD4, 2048, 1024));
	    temp_x4 = adc_ad8495_conv(avg_ch2);
	    while (temp_x4 <150) {
	    	temp_x4 = adc_ad8495_conv(avg_ch2);
	    	temp_degc = temp_x4 / 4;
	    	chprintf((BaseChannel *)&SD1, "%d: Preheat 50%  Temp: %d deg\r\n", seconds_counter, temp_degc);
	    	chThdSleepMilliseconds(1000);
	    	seconds_counter++;
	    	}

    	//Turn off fan
		pwmEnableChannelI(&PWMD4, 0, PWM_FRACTION_TO_WIDTH(&PWMD4, 2048, 0));
	    preheat = FALSE;
	    running = TRUE;		//start a run after button is pressed
	    seconds_counter=0;	//reset run counter
      }

    chThdSleepMilliseconds(1000);


    //Grab the temp values we care about
    temp_x4 = adc_ad8495_conv(avg_ch2);
    target_x4 = desired_temps_x4[seconds_counter];
    error_x4 = temp_x4 - target_x4;

    //Create values to display to the user
    temp_degc = temp_x4 / 4;
    target_degc = target_x4 / 4;
    error_degc = error_x4 / 4;

    //PID related

    if(running == TRUE)
    	{
    	//increment ticks counter
        clk_ticks+= 1000;

    	//Active running case
    	if (seconds_counter < 170) {  //We have our temp profile defined for 203 seconds, use first 180 then broil
    		//update target values
    		heatPID.targetPos = target_x4;

    		//recalc PWM values
    		heat_duty = PIDUpdatePid(&heatPID, temp_x4, clk_ticks);

    		//debugging feedback over serial
    		chprintf((BaseChannel *)&SD1, "%d Temp: %d deg: Target: %d deg Error: %d deg Fan: %u Heat %u\r\n", seconds_counter, temp_degc, target_degc, error_degc, fan_duty, heat_duty);

    	    /* Changes the channels pulse width, the change will be effective
    	       starting from the next cycle.*/
    	    pwmEnableChannelI(&PWMD4, 1, PWM_FRACTION_TO_WIDTH(&PWMD4, 2048, heat_duty));
    	}
    	//End of cycle case
    	else {
    		//ensure we hit target temp
    		pwmEnableChannel(&PWMD4, 1, PWM_FRACTION_TO_WIDTH(&PWMD4, 2048, 2048));  //heat on max
    		while (temp_x4 < 760) {
    			temp_x4 = adc_ad8495_conv(avg_ch2);
    			temp_degc = temp_x4 / 4;
    			chprintf((BaseChannel *)&SD1, "%d: BROIL Temp: %d deg\r\n", seconds_counter, temp_degc);
    			chThdSleepMilliseconds(1000);
    		}

    		//begin cooldown
    			pwmEnableChannel(&PWMD4, 1, PWM_FRACTION_TO_WIDTH(&PWMD4, 2048, 0));  //heat off
    		    pwmEnableChannel(&PWMD4, 0, PWM_FRACTION_TO_WIDTH(&PWMD4, 2048, 512));	//fan on low duty
    		    while (temp_x4 > 720) {
    		    	 temp_x4 = adc_ad8495_conv(avg_ch2);
    		    	 temp_degc = temp_x4 / 4;
    		    	 chprintf((BaseChannel *)&SD1, "%d: Fan 25% Temp: %d deg\r\n", seconds_counter, temp_degc);
    		    	 chThdSleepMilliseconds(1000);
    		    	 }
    		    pwmEnableChannel(&PWMD4, 0, PWM_FRACTION_TO_WIDTH(&PWMD4, 2048, 2048));	//fan on full blast
    		    while (temp_x4 > 160) {
    		    	 temp_x4 = adc_ad8495_conv(avg_ch2);
    		    	 temp_degc = temp_x4 / 4;
    		    	 chprintf((BaseChannel *)&SD1, "%d: Fan 100% Temp: %d deg\r\n", seconds_counter, temp_degc);
    		    	 chThdSleepMilliseconds(1000);
    		    	 }
    		    pwmEnableChannel(&PWMD4, 0, PWM_FRACTION_TO_WIDTH(&PWMD4, 2048, 0));	//fan off
    			chprintf((BaseChannel *)&SD1, "Run Finished!\r\n");
    		    seconds_counter = 0;
    		    clk_ticks = 0;
    		    resetPIDStruct(&heatPID, clk_ticks);
    		    running = FALSE;
    		}//end if - else (seconds_counter)
    	}//end if (running == TRUE)
  }
}
