#ifndef KVETINAC_ANALOG_HPP
#define KVETINAC_ANALOG_HPP

//#include "../avrlib/adc.hpp"
#include "filter.hpp"

#define CS_L 0
#define CS_R 2
#define BAT  3
#define ADC4 4
#define ADC5 5
#define ADC6 6
#define ADC7 7
#define TEST_1V23 31

#define REF_AVCC (0<<REFS1) //default
#define REF_2V56 (1<<REFS1)
#define LEFT_ADJUST_RESULT (1<<ADLAR)
#define RIGHT_ADJUST_RESULT (0<<ADLAR) //default


#ifndef KVETINAC_ANALOG_SLOTS
#define KVETINAC_ANALOG_SLOTS 8
#endif

#ifndef KVETINAC_ANALOG_NEXT_PRESCALLER
#define KVETINAC_ANALOG_NEXT_PRESCALLER 2
#endif

class analog_t
{
public:
	typedef uint16_t value_type;
	typedef uint8_t index_type;
// #ifdef KVETINAC_ANALOG_LEFT_ADJUST
// 	typedef uint32_t filter_value_type;
// #else
// 	typedef value_type filter_value_type;
// #endif
	typedef value_type filter_value_type;
	typedef uint32_t filter_internal_type;
	typedef filter_base<filter_value_type/*, filter_internal_type*/> filter_type;

	analog_t (index_type settings, filter_type& filter/* = no_filter<filter_value_type>()*/)
		:m_index(s_active), m_settings(0), m_filter(filter), m_value_available(false)
	{
		if(m_index == KVETINAC_ANALOG_SLOTS)
			return;
		m_settings = settings|(1<<REFS0);
		s_slot[m_index] = this;
		++s_active;
	}
	
	~analog_t()
	{
		if(m_settings == 0)
			return;
		m_settings = 0;
		s_slot[m_index] = 0;
		--s_active;
	}		
		
	//operator value_type () { return m_filter.output(); }
	value_type operator ()() { return m_filter.output(); }
		
	bool new_value_available()
	{
		if(!m_value_available)
			return false;
		m_value_available = false;
		return true;
	}
	
	value_type unfiltered_value() { return m_value; }

	filter_type* filter() { return &m_filter; }
	
	void process(value_type value)
	{
		//format(data, "ap%:%5\n") % m_index % value;
		m_filter.input(value);
		m_value_available = true;
		m_value = value;
	}
	
	static uint8_t get_active()
	{
		return s_active;
	}
	
	static void init()
	{
		ADMUX = s_slot[0] != 0 ? s_slot[0]->m_settings : (1<<REFS0);
		ADCSRA = c_adcsra | (1<<ADSC);
		while(!convertion_complete()) {};
	}
	
	static void next()
	{
		if(converting())
			return;
		//format(data, "next:%2:") % s_index;
		if(s_slot[s_index] != 0)
		{
			s_slot[s_index]->process(ADC);
			//format(data, " value:%4;") % ADC;
		}			
		if(++s_index >= KVETINAC_ANALOG_SLOTS)
			s_index = 0;
		//format(data, " ++%2:") % s_index;
		if(s_slot[s_index] != 0)
		{
			ADMUX = s_slot[s_index]->m_settings;
			//format(data, " settings:%3x;") % ADMUX;
			ADCSRA = c_adcsra | (1<<ADSC);
		}
		//send(data, "\n");
	}
	
private:

	static bool converting() { return ((ADCSRA & (1<<ADSC)) != 0); }
	static bool convertion_complete()
	{
		if((ADCSRA & (1<<ADIF)) == 0)
			return false;
		ADCSRA = c_adcsra;
		return true;
	}

	index_type m_index;
	index_type m_settings;
	filter_type& m_filter;
	volatile bool m_value_available;
	volatile value_type m_value;
	
	static analog_t* s_slot[KVETINAC_ANALOG_SLOTS];
	static index_type s_index;
	static index_type s_active;
	
	static const uint8_t c_adcsra = (1<<ADEN)  | (1<<ADIF) | 7;
};

analog_t* analog_t::s_slot[KVETINAC_ANALOG_SLOTS] = {0};
analog_t::index_type analog_t::s_index = 0;
analog_t::index_type analog_t::s_active = 0;

//weighted_moving_average<analog_t::filter_value_type, analog_t::filter_internal_type> battery_voltage_filter(16, 1, 16, 10);
no_filter<analog_t::value_type> battery_voltage_filter;
analog_t battery_voltage(BAT, battery_voltage_filter);
//weighted_moving_average<analog_t::filter_value_type, analog_t::filter_internal_type> left_motor_current_filter(16, 1, 16, 10);
no_filter<analog_t::value_type> left_motor_current_filter;
analog_t left_motor_current(CS_L, left_motor_current_filter);
//weighted_moving_average<analog_t::filter_value_type, analog_t::filter_internal_type> right_motor_current_filter(16, 1, 16, 10);
no_filter<analog_t::value_type> right_motor_current_filter;
analog_t right_motor_current(CS_R, right_motor_current_filter);

typedef analog_t analog;

#endif