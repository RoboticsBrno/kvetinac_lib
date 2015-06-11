#ifndef KVETINAC_SERVOS_HPP
#define KVETINAC_SERVOS_HPP

#ifndef KVETINAC_INCLUDED
#define KVETINAC_INCLUDED
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

#include "avrlib/timer3.hpp"
#include "avrlib/math.hpp"

using avrlib::timer3;
using avrlib::clamp;

#ifndef KVETINAC_ENABLE_SERVOS
#define KVETINAC_ENABLE_SERVOS true
#endif

class servo_t
{
	public:
	typedef uint16_t precision_value_type;
	typedef int8_t value_type;
	typedef uint8_t index_type;
	
	static void init(bool en = true)
	{
		m_current = 0;
		for(index_type i = 0; i != number_of_servos_c; ++i)
		m_value[i] = mid_value_c;
		DDRE |= (1<<5); //SEN
		DDRD |= 0xE0; //SA0-2
		timer3::ocra::value(timer_period_c);
		timer3::mode(avrlib::timer_mode_pwm_ocra);
		enable(en);
		timer3::ocrc::value(mid_value_c);
		timer3::ocrc::interrupt(true);
		timer3::tov_interrupt(true);
		timer3::clock_source(avrlib::timer_fosc_1);
	}
	
	static void enable(bool en = true)
	{
		if(en)
		timer3::ocrc::mode(avrlib::timer_ocr_mode_positive);
		else
		timer3::ocrc::mode(avrlib::timer_ocr_mode_disconnected);
	}
	
	static inline void process_comp()
	{
		PORTD = (PORTD & 0x1F) | (m_current<<5);
	}
	
	static inline void process_ovf()
	{
		timer3::ocrc::value(m_value[m_current]);
		++m_current;
		m_current &= 7;
	}
	
	static void set(index_type index, precision_value_type value)
	{
		if(index >= number_of_servos_c)
		return;
		_set(translate_index(index), clamp(value, min_value_c, max_value_c));
	}
	
	static precision_value_type get(index_type index)
	{
		if(index >= number_of_servos_c)
		return 0;
		return m_value[translate_index(index)];
	}
	
	servo_t(index_type index, precision_value_type servo_min_value = min_value_c, precision_value_type servo_max_value = max_value_c)
		:m_index(translate_index(index)), m_servo_min_value(servo_min_value), m_servo_max_value(servo_max_value)
	{
	}
	
	void set(precision_value_type value)
	{
		_set(m_index, clamp(value, m_servo_min_value, m_servo_max_value));
	}
	
	precision_value_type get()
	{
		return m_value[m_index];
	}
	
	void set_min(precision_value_type value)
	{
		m_servo_min_value = value;			
	}
	
	void set_max(precision_value_type value)
	{
		m_servo_max_value = value;
	}
	
	static const precision_value_type timer_period_c = 40000;	//2.5ms
	static const precision_value_type max_value_c = 38400;		//0.6ms
	static const precision_value_type mid_value_c = 24000;		//1.5mm
	static const precision_value_type min_value_c = 9600;		//2.4mm

	static const uint8_t number_of_servos_c = 8;
	
	private:

	static void _set(index_type index, precision_value_type value)
	{
		if(value != m_value[index])
		{
			cli();
			m_value[index] = value;
			sei();
		}
	}
	
	static index_type translate_index(index_type index)
	{
	static const index_type translation_table[number_of_servos_c] = {7, 3, 1, 5, 0, 4, 2, 6};
	return translation_table[index];
}

const index_type m_index;

precision_value_type m_servo_min_value;
precision_value_type m_servo_max_value;

static index_type m_current;
static volatile precision_value_type m_value[number_of_servos_c];

};

servo_t::index_type servo_t::m_current;
volatile servo_t::precision_value_type servo_t::m_value[servo_t::number_of_servos_c];

ISR(TIMER3_COMPC_vect)
{
	servo_t::process_comp();
}

typedef servo_t servo;

#endif