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

#include "avrlib/math.hpp"

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

using avrlib::clamp;

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

// Delays for for the specified nubmer of microseconds.
inline void delayMicroseconds(uint16_t microseconds)
{
	__asm__ volatile (
	"1: push r22"     "\n\t"
	"   ldi  r22, 4"  "\n\t"
	"2: dec  r22"     "\n\t"
	"   brne 2b"      "\n\t"
	"   pop  r22"     "\n\t"
	"   sbiw %0, 1"   "\n\t"
	"   brne 1b"
	: "=w" ( microseconds )
	: "0" ( microseconds )
	);
}


inline void delay(uint16_t ms)
{
	while (ms--)
		delayMicroseconds(1000);
}

stopwatch time;

uint32_t getTicksCount()
{
	return time() / 16000UL;
}

void resetTicks()
{
	time.restart();
}

typedef pin<porte, 0> pin_usart0_rx;
typedef pin<porte, 1> pin_usart0_tx;

#ifndef UART0_RX_BUFF
#define UART0_RX_BUFF 128
#endif

#ifndef UART0_TX_BUFF
#define UART0_TX_BUFF 128
#endif

//typedef async_usart<usart0, UART0_RX_BUFF, UART0_TX_BUFF, kvetinac_bootseq> rs232_t;
//rs232_t rs232(115200UL, true);

template <typename Usart, int RxBufferSize, int TxBufferSize, typename Bootseq, typename Overflow>
class rs232_template : public async_usart <Usart, RxBufferSize, TxBufferSize, Bootseq, Overflow>
{
	public:
	typedef Usart usart_type;

	rs232_template()
	{
	}
	
	template <typename T1>
	rs232_template(T1 const & t1)
	{
		m_usart.open(t1);
	}

	template <typename T1, typename T2>
	rs232_template(T1 const & t1, T2 const & t2)
	{
		m_usart.open(t1, t2);
	}
	
	char get()
	{
		return this->read();
	}

	bool peek(char & ch)
	{
		if (this->empty())
			return false;

		ch = this->read();
		return true;
	}

	void sendCharacter(char ch)
	{
		this->write(ch);
	}

	void send(const char * str)
	{
		avrlib::send(*this, str);
	}

// 	void sendHexByte(uint8_t byte)
// 	{
// 		static const char hexdigits[] = "0123456789ABCDEF";
// 		this->sendCharacter(hexdigits[byte >> 4]);
// 		this->sendCharacter(hexdigits[byte & 0x0f]);
// 	}
// 
// 	template <typename T>
// 	void sendNumberHex(T n, uint8_t width = 0)
// 	{
// 		char buf[32];
// 		uint8_t len = 0;
// 
// 		if (n != 0)
// 		{
// 			T a = abs(n);
// 
// 			while (a > 0)
// 			{
// 				buf[len++] = '0' + (a & 0x0f);
// 				a = a >> 4;
// 			}
// 
// 			if (n < 0)
// 			buf[len++] = '-';
// 		}
// 		else
// 		buf[len++] = '0';
// 
// 		for (; width > len; --width)
// 		m_txbuf.push(' ');
// 		for (; len > 0; --len)
// 		m_txbuf.push(buf[len-1]);
// 		UCSR0B |= (1<<UDRIE0);
// 	}
// 
	template <typename T>
	void sendNumber(T n, uint8_t width = 0)
	{
		avrlib::send_int(*this, n, width);
	}

	template <typename T>
	void dumpNumber(T n)
	{
		sendNumber(n);
		sendCharacter(' ');
	}

	void wait()
	{
		this->flush();
	}

	bool txempty() const
	{
		return this->empty();
	}
	 
private:
	usart_type m_usart;
};

typedef rs232_template<usart0, UART0_RX_BUFF, UART0_TX_BUFF, kvetinac_bootseq, uint32_t> rs232_t;
rs232_t rs232(115200UL, true);

ISR(USART0_RX_vect)
{
	rs232.intr_rx();
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
	com.intr_rx();
}

typedef pin<porte, 4> pin_stop_btn;


inline bool isPressed(uint8_t buttons)
{
	buttons = 1 << buttons;
	PORTA |= buttons;
	return !(PINA & buttons);
}

inline uint8_t waitForPress(uint8_t buttons)
{
	buttons = 1 << buttons;
	while(PINA & buttons)
	{
		delay(5);
	}
	return ((~PINA) & buttons);       // return the pressed button(s)
}

inline uint8_t waitForRelease(uint8_t buttons)
{
	buttons = 1 << buttons;
	while(!(PINA & buttons))
	{
		delay(5);
	}
	return (PINA & buttons);          // return the pressed button(s)
}

inline void waitForButton(uint8_t buttons)
{
	do
	{
		while(!isPressed(buttons))
		{
			delay(5);
		}
		delay(5);
	} while(!isPressed(buttons));
	
	delay(50);
	
	do
	{
		while(isPressed(buttons))
		{
			delay(5);
		}
		delay(5);
	} while(isPressed(buttons));
}

/**
 * Nastavi vykon leveho motoru
 * @param speed vykon od -255 do +255
 */
void setLeftMotor(int16_t speed)
{
	speed = (clamp(speed, -250, 250) << 1);
	
	left_motor::power(speed);
}

/**
 * Nastavi vykon praveho motoru
 * @param speed vykon od -255 do +255
 */
void setRightMotor(int16_t speed)
{
	speed = (clamp(speed, -250, 250) << 1);
	
	right_motor::power(speed);
}

/**
 * Nastavi vykon motoru dle zadaneho ID
 * @param motor_id cislo motoru (0 = levy, 1 = pravy)
 * @param speed vykon od -255 do +255
 */
void setMotorPowerID(uint8_t motor_id, int16_t speed)
{
	if(motor_id == 0)
		setLeftMotor(speed);
	else if(motor_id == 1)
		setRightMotor(speed);
}

/**
 * Nastavi vykon motoru
 * @param left vykon od -255 do +255
 * @param right vykon od -255 do +255
 */
void setMotorPower(int16_t left, int16_t right)
{
	left = left << 1;//clamp(left, -250, 250) << 1;
	right = right << 1; //clamp(right, -250, 250) << 1;
	
	left_motor::power(left);
	right_motor::power(right);
}


void run(void);

void init()
{
	time.restart();
	
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
	delay(1);
	sei();
	init();
	//wait(msec(10));
	run();
	for(;;)
	{}
}
		
#endif