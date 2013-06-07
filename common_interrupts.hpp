#ifndef KVETINAC_INTERRUPTS_HPP
#define KVETINAC_INTERRUPTS_HPP

#include <avr/io.h>
#include <avr/interrupt.h>

ISR(TIMER3_OVF_vect)
{
	#ifdef KVETINAC_TIME_HPP
		base_timer.tov_interrupt();
	#endif
	#ifdef KVETINAC_SERVOS_HPP
		servo_t::process_ovf();
	#endif
}

ISR(TIMER1_OVF_vect)
{
	#ifdef KVETINAC_MOTORS_HPP
		motors_int_process();
	#endif
	#ifdef KVETINAC_ANALOG_HPP
		static uint8_t prescaller = KVETINAC_ANALOG_NEXT_PRESCALLER;
		if(--prescaller == 0)
		{
			analog_t::next();
			prescaller = KVETINAC_ANALOG_NEXT_PRESCALLER;
		}
	#endif
}

#endif