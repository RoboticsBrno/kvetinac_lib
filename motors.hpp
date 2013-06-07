#ifndef KVETINAC_MOTORS_HPP
#define KVETINAC_MOTORS_HPP

#ifndef KVETINAC_MOTORS_PWM_RESOLUTION
#define KVETINAC_MOTORS_PWM_RESOLUTION 512
#endif

#ifndef KVETINAC_MOTORS_INVERT_LEFT
#define KVETINAC_MOTORS_INVERT_LEFT false
#endif
#ifndef KVETINAC_MOTORS_INVERT_RIGHT
#define KVETINAC_MOTORS_INVERT_RIGHT false
#endif

#include "avrlib/portb.hpp"
#include "avrlib/porte.hpp"
#include "avrlib/portf.hpp"
#include "avrlib/portg.hpp"
#include "avrlib/pin.hpp"
#include "avrlib/timer1.hpp"

using avrlib::portb;
using avrlib::porte;
using avrlib::portf;
using avrlib::portg;
using avrlib::pin;
using avrlib::timer1;

template < typename in_a, typename in_b, typename en_a, typename en_b, typename OCR , bool invert = false>
struct motor_t
{
	static void power(int16_t p, bool brakes = false)
	{
		if(invert)
			p = -p;
		if (p <= 0)
		{
			if (!brakes)
				in_a::set();
			else
				in_a::clear();
			in_b::clear();
		}
		else if (p > 0)
		{
			in_b::set();
			in_a::clear();
		}

		uint16_t abs_current = p < 0? -p: p;
		if (abs_current > KVETINAC_MOTORS_PWM_RESOLUTION)
			abs_current = KVETINAC_MOTORS_PWM_RESOLUTION;
		OCR::value(abs_current);
	}

	static int16_t power()
	{
		int16_t p =  in_b::get()>in_a::get()?OCR::value():-OCR::value();
		if(invert)
			p = -p;
		return p;
	}

	static void enable(bool en = true)
	{
		en_a::set(en);
		en_a::output(!en);
		en_b::set(en);
		en_b::output(!en);
	}

	static void brake()
	{
		in_a::set();
		in_b::set();
		OCR::value(0);
	}
	
	static bool ok()
	{
		return en_a::value() && en_b::value();
	}
	
};

typedef pin<porte, 6> pin_motor_in_la;
typedef pin<porte, 2> pin_motor_in_lb;
typedef pin<porte, 3> pin_motor_diag_la;
typedef pin<portf, 1> pin_motor_diag_lb;
typedef pin<portb, 5> pin_motor_pwm_l; // OC1A

typedef pin<portg, 4> pin_motor_in_ra;
typedef pin<porte, 7> pin_motor_in_rb;
typedef pin<portg, 3> pin_motor_diag_ra;
typedef pin<portb, 4> pin_motor_diag_rb;
typedef pin<portb, 6> pin_motor_pwm_r; // OC1B

typedef motor_t<pin_motor_in_la, pin_motor_in_lb, pin_motor_diag_la, pin_motor_diag_lb, timer1::ocra, KVETINAC_MOTORS_INVERT_LEFT> left_motor_type;
typedef motor_t<pin_motor_in_ra, pin_motor_in_rb, pin_motor_diag_ra, pin_motor_diag_rb, timer1::ocrb, KVETINAC_MOTORS_INVERT_RIGHT> right_motor_type;

typedef left_motor_type left_motor;
typedef right_motor_type right_motor;

void motors_init()
{
	pin_motor_in_la::output(true);
	pin_motor_in_lb::output(true);
	pin_motor_in_ra::output(true);
	pin_motor_in_rb::output(true);
	pin_motor_pwm_l::output(true);
	pin_motor_pwm_r::output(true);
	pin_motor_diag_la::set_high();
	pin_motor_diag_lb::set_high();
	pin_motor_diag_ra::set_high();
	pin_motor_diag_rb::set_high();
	timer1::icr::value(KVETINAC_MOTORS_PWM_RESOLUTION);
	timer1::mode(avrlib::timer_mode_pwmp_icr);
	timer1::ocra::mode(avrlib::timer_ocr_mode_positive);
	timer1::ocrb::mode(avrlib::timer_ocr_mode_positive);
	timer1::tov_interrupt(true);
	timer1::clock_source(avrlib::timer_fosc_1);
}

void motors_int_process()
{
	
}

#endif