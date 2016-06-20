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
		for(index_type i = 0; i != number_of_servos_c; ++i)
			m_value[i] = mid_value_c;
		DDRE |= (1<<4) | (1<<5); //OC3B, OC3C
//		timer3::ocra::value(timer_period_c);
//		timer3::mode(avrlib::timer_mode_pwm_ocra);
//		enable(en);
// 		timer3::ocrc::value(mid_value_c);
// 		timer3::ocrc::interrupt(true);
// 		timer3::tov_interrupt(true);
// 		timer3::clock_source(avrlib::timer_fosc_1);
		
		timer3::icr::value(40000);
		timer3::mode(avrlib::timer_mode_pwm_icr);
		enable(true);
		timer3::ocrb::value(3000);
		timer3::ocrc::value(3000);
		timer3::tov_interrupt(true);
		timer3::clock_source(avrlib::timer_fosc_8);
	}
	
	static void enable(bool en = true)
	{
		if(en)
		{
			timer3::ocrb::mode(avrlib::timer_ocr_mode_positive);
			timer3::ocrc::mode(avrlib::timer_ocr_mode_positive);
		}
		else
		{
			timer3::ocrb::mode(avrlib::timer_ocr_mode_disconnected);
			timer3::ocrc::mode(avrlib::timer_ocr_mode_disconnected);
		}
	}
	
	static void _set(index_type index, precision_value_type value)
	{
		if (index == 0)
			avrlib::timer3::ocrc::value(value);
		else
			avrlib::timer3::ocrb::value(value);
	}
	
	static precision_value_type _get (index_type index)
	{
		if (index == 0)
			return avrlib::timer3::ocrc::value();
		else
			return avrlib::timer3::ocrb::value();
	}
	
	precision_value_type get()
	{
		return _get(m_index);
	}
	
	void set(precision_value_type value)
	{
		_set(m_index, clamp(value, m_servo_min_value, m_servo_max_value));
	}
		
	servo_t(index_type index, precision_value_type servo_min_value = min_value_c, precision_value_type servo_max_value = max_value_c)
		:m_index((index)), m_servo_min_value(servo_min_value), m_servo_max_value(servo_max_value)
	{
	}
	
	static const precision_value_type timer_period_c = 40000;
	static const precision_value_type max_value_c = 5000;
	static const precision_value_type mid_value_c = 3000;		
	static const precision_value_type min_value_c = 1000;		

	static const uint8_t number_of_servos_c = 2;
	
private:
	const index_type m_index;

	precision_value_type m_servo_min_value;
	precision_value_type m_servo_max_value;

	static index_type m_current;
	static volatile precision_value_type m_value[number_of_servos_c];

};

servo_t::index_type servo_t::m_current;
volatile servo_t::precision_value_type servo_t::m_value[servo_t::number_of_servos_c];

typedef servo_t servo;

#endif