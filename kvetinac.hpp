#ifndef KVETINAC_HPP
#define KVETINAC_HPP

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <util/delay.h>

#include "avrlib/porta.hpp"
#include "avrlib/portb.hpp"
#include "avrlib/portc.hpp"
#include "avrlib/portd.hpp"
#include "avrlib/porte.hpp"
#include "avrlib/portf.hpp"
#include "avrlib/portg.hpp"
#include "avrlib/pin.hpp"

#include "avrlib/usart_base.hpp"
#include "avrlib/async_usart.hpp"
#include "avrlib/sync_usart.hpp"
#include "avrlib/usart0.hpp"
#include "avrlib/usart1.hpp"
#include "avrlib/bootseq.hpp"

#include "avrlib/format.hpp"

#include "kvetinac_bootseq.hpp"

using avrlib::porta;
using avrlib::portb;
using avrlib::portc;
using avrlib::portd;
using avrlib::porte;
using avrlib::portf;
using avrlib::portg;
using avrlib::pin;

using avrlib::usart0;
using avrlib::usart1;
using avrlib::async_usart;
using avrlib::sync_usart;

typedef pin<porte, 0> pin_usart0_rx;
typedef pin<porte, 1> pin_usart0_tx;

#ifndef UART0_RX_BUFF
#define UART0_RX_BUFF 128
#endif

#ifndef UART0_TX_BUFF
#define UART0_TX_BUFF 128
#endif

typedef async_usart<usart0, UART0_RX_BUFF, UART0_TX_BUFF, kvetinac_bootseq> debug_t;
debug_t debug(115200UL, true);

ISR(USART0_RX_vect)
{
	debug.process_rx();
}

typedef pin<portd, 2> pin_usart1_rx;
typedef pin<portd, 3> pin_usart1_tx;

#ifndef UART1_RX_BUFF
#define UART1_RX_BUFF 128
#endif

#ifndef UART1_TX_BUFF
#define UART1_TX_BUFF 128
#endif

typedef async_usart<usart1, UART1_RX_BUFF, UART1_TX_BUFF, avrlib::nobootseq> com_t;
com_t com(115200UL, true);

ISR(USART1_RX_vect)
{
	com.process_rx();
}

typedef pin<porte, 4> pin_stop_btn;

#ifndef KVETINAC_INCLUDED
#include "servos.hpp"
#include "time.hpp"
#include "filter.hpp"
#include "scale.hpp"
#include "analog.hpp"
#include "motors.hpp"
#include "common_interrupts.hpp"
#include "encoders.hpp"
#endif

void run(void);

void init()
{
	#ifdef KVETINAC_SERVOS_HPP
		servo_t::init(KVETINAC_ENABLE_SERVOS);
	#endif
	#ifdef KVETINAC_ANALOG_HPP
		analog_t::init();
	#endif
	#ifdef KVETINAC_MOTORS_HPP
		motors_init();
	#endif
	#ifdef KVETINAC_ENCODERS_HPP
		encoders_init();
	#endif
}

void kvetinac_bootseq_reset()
{
	cli();
	encoders_comm.stop();

	#if defined(WDTCR)
	# if defined(WDCE)
	WDTCR = (1<<WDCE)|(1<<WDE);
	# else
	WDTCR = (1<<WDTOE)|(1<<WDE);
	# endif
	WDTCR = (1<<WDE);
	#elif defined(WDTCSR)
	WDTCSR = (1<<WDCE)|(1<<WDE);
	WDTCSR = (1<<WDE);
	#elif __AVR_ARCH__ >= 100 /*xmega*/
	CCP = CCP_IOREG_gc;
	RST.CTRL = RST_SWRST_bm;
	#else
	# error Unsupported Watchdog timer interface.
	#endif
	for (;;)
	{
	}
}

int main(void)
{
	pin_usart0_rx::set_high(); //pull-up
	pin_usart1_rx::set_high(); //pull-up
	init();
	sei();
	//wait(msec(10));
	run();
	for(;;)
	{}
}
		
#endif