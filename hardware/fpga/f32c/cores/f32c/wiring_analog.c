/* EMARD */

#include "Arduino.h"
#include "wiring_analog.h"
#include <dev/io.h>

__BEGIN_DECLS

static uint8_t analog_write_resolution_bits = 8;
/* old arduino uses 490 Hz */
/* new arduino uses 980 Hz */
static uint32_t analog_write_frequency = 980;

const struct pwm_enable_bitmask_s pwm_enable_bitmask[] = VARIANT_PWM_CHANNEL_ENABLE;

/* todo: support analogWriteResolution(bits)
** default is 8 for old arduino,
** new arduino can use up to 12
*/
void analogWriteResolution(int res)
{
  analog_write_resolution_bits = res;
}

void analogWriteFrequency(int freq)
{
  analog_write_frequency = freq;
}

/* todo: handle the pin */
void analogWritePhase(uint32_t pin, uint32_t phase)
{
  int8_t pwm_channel;
  volatile uint32_t *start, *stop;

  if(pin >= variant_pin_map_size)
    return;

  pwm_channel = variant_pin_map[pin].pwm;
  if(pwm_channel != OCP_NONE)
  {
    start  = &EMARD_TIMER[pwm_enable_bitmask[pwm_channel].ocp_start];
    stop   = &EMARD_TIMER[pwm_enable_bitmask[pwm_channel].ocp_stop];
    
    *stop  = phase + (*stop - *start);
    *start = phase;
    
    EMARD_TIMER[TC_APPLY] = pwm_enable_bitmask[pwm_channel].apply;
  }
}

/* setup the common parameters (why isn't it called at startup?)
*/
void analogOutputInit( void )
{
}

void analogWrite(uint32_t ulPin, uint32_t ulValue)
{
    (*(volatile uint32_t *) IO_PWM_DUTY) = ulValue;
}

void analogReference( eAnalogReference ulMode ) 
{
}

uint32_t analogRead(uint32_t ulPin)
{
  /* TODO
     external parallel RC circuit to GND,
     3 state output hardware that
     sets pin to output high, (charges C),
     set pin to input (high impedance, no pullup), 
     and captures time when input becomes low (C discharges),
     time is captured in a I/O mapped register
     which represents analog value of RC circuit
     
     similar as input capture, but with 3-state output and input
     on the same pin
  */
  uint32_t r = 0;

#if defined(IO_ADC_A0)
  /* TO-DONE: The above was done on FleaFPGA-Uno board and this code reads the result. :-) */
    volatile uint32_t *adc = (volatile uint32_t *)IO_ADC_A0;
	volatile uint32_t *port = (volatile uint32_t *) IO_GPIO_CTL;
	
	if (ulPin >= A0 && ulPin <= A5)
	{
		*port &= ~(1<<variant_pin_map[ulPin].bit_pos);	// make sure set to INPUT
		int16_t v = adc[ulPin-A0];
		if (v < IO_ADC_MINVAL)
			v = IO_ADC_MINVAL;
		if (v > IO_ADC_MAXVAL)
			v = IO_ADC_MAXVAL;
		r = (v - IO_ADC_MINVAL) * 1023 / (IO_ADC_MAXVAL - IO_ADC_MINVAL);	// scale 0 to 1023
	}
#endif

  return r;
}

/* input capture setting */
uint32_t setInputCapture(uint32_t ulPin)
{
  int8_t icp_channel;
  
  if(ulPin >= variant_pin_map_size)
    return 0;
    
  icp_channel = variant_pin_map[ulPin].icp;
  if(icp_channel != ICP_NONE)
  {

    #if 0
    /* input caputre setting
    ** this code doesn't belong here
    ** here belongs RC co
    
    ** initializing ICP engine to capture all events
    */
    EMARD_TIMER[TC_CONTROL] &= pwm_enable_bitmask[icp_channel].control_and;
    EMARD_TIMER[TC_CONTROL] |= pwm_enable_bitmask[icp_channel].control_or;
    EMARD_TIMER[TC_APPLY] = pwm_enable_bitmask[icp_channel].apply;
    /*
    todo:
    [ ] make RC component, and 3 state pin
    [ ] create icp control array
    [ ] 
    */
    #endif

  }
  return 0;
}

__END_DECLS
