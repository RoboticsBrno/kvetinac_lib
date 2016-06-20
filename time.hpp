#ifndef KVETINAC_TIME_HPP
#define KVETINAC_TIME_HPP

#ifndef KVETINAC_INCLUDED
#define KVETINAC_INCLUDED
#endif

#ifndef KVETINAC_BASE_TIMER_VALUE_TYPE
#define KVETINAC_BASE_TIMER_VALUE_TYPE uint32_t
#endif

#include "avrlib/timer3.hpp"
#include "avrlib/counter.hpp"
#include "avrlib/stopwatch.hpp"

using avrlib::timer3;

typedef avrlib::counter<timer3, KVETINAC_BASE_TIMER_VALUE_TYPE, uint32_t, false> base_timer_type;
base_timer_type base_timer;

struct stopwatch
	:avrlib::stopwatch<base_timer_type>
{
	stopwatch(bool run = true)
		:avrlib::stopwatch<base_timer_type>(base_timer)
	{
		if(run)
			return;
		stop();
		clear();
	}
};

struct timeout
	:avrlib::timeout<base_timer_type>
{
	timeout(avrlib::timeout<base_timer_type>::time_type timeout)
		:avrlib::timeout<base_timer_type>(base_timer, timeout)
	{
	}
};

void wait(base_timer_type::time_type time)
{
	avrlib::wait(base_timer, time);
}

template <typename Process>
void wait(base_timer_type::time_type time, Process process)
{
	avrlib::wait(base_timer, time, process);
}

template <typename Process>
void wait(base_timer_type::time_type time, Process process, int)
{
	avrlib::wait(base_timer, time, process, 0);
}

#define  sec(value) (2000000UL*value)
#define msec(value) (2000UL*value)
#define usec(value) (2UL*value)


#endif