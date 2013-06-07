#ifndef KVETINAC_ENCODERS_HPP
#define KVETINAC_ENCODERS_HPP

#include <avr/io.h>
#include <stdint.h>
#include <math.h>

#include "avrlib/portg.hpp"
#include "avrlib/pin.hpp"
#include "avrlib/buffer.hpp"

using avrlib::pin;

static const uint16_t c_automatic_gain_control_register  = ((1<<15)|(0x3FF8<<1)|1)^1;
static const uint16_t c_angular_data_register            = ((1<<15)|(0x3FFF<<1)|0)^1;
static const uint16_t c_clear_error_flag_register        = ((1<<15)|(0x3380<<1)|1)^1;
static const uint16_t c_sw_reset_cmd                     = ((0x3C00<<1)|1)^1;
static const uint16_t c_sw_reset_data                    = 0;

class encoder_t
{
public:
    encoder_t(uint8_t cs_pin)
    {
        m_value = 0;
        m_value_last = 0;
        m_distance = 0;
        m_distance_last = 0;
        m_speed = 0;
        m_speed_last = 0;
        m_accel = 0;
        m_state = 0;
        m_agc = 0;
        m_cs_pin = cs_pin;
        m_clr_err_req = false;
    }

    void clear()
    {
        m_state = 0;
        m_value_last = m_value;
        m_distance = 0;
        m_distance_last = 0;
        m_speed = 0;
        m_speed_last = 0;
        m_accel = 0;
    }

    void compute()
    {
        int16_t delta = m_value - m_value_last;
        if(abs(delta) > 2048)
        {
            if (delta < 0)
                delta = 4095 - m_value_last + m_value;
            else
                delta = -(4095 - m_value + m_value_last);
        }
        m_value_last = m_value;
        if(abs(delta) < 6)
            delta = 0;
        m_distance += delta;
        m_speed = m_distance - m_distance_last;
        m_distance_last = m_distance;
        m_accel = m_speed - m_speed_last;
        m_speed_last = m_speed;
    }

    void clearErrFlag() { m_clr_err_req = true; }
    bool takeErrReqStatus()
    {
        if(!m_clr_err_req)
            return false;
        else
        {
            m_clr_err_req = false;
            return true;
        }
    }

    void cmd_result(const uint16_t& cmd, int16_t r)
    {
        switch(cmd)
        {
            case c_angular_data_register:
                setValue(r);
                break;
            case c_automatic_gain_control_register:
                setAgc(r);
                break;
            default:
                break;
        }
    }

    void toggle_cs(bool high)
    {
        // FIXME: this should be handled better
        if(high) PORTG |= (1 << m_cs_pin);
        else     PORTG &= ~(1 << m_cs_pin);
    }

    int16_t value() const { return m_value; }
    int16_t speed() const { return m_speed; }
    int32_t distance() const { return m_distance; }
    int16_t accel() const { return m_accel; }
    uint8_t agc() const { return m_agc; }
    uint8_t state() const { return m_state; }

private:
    inline void setValue(int16_t& r)
    {
        m_state = r & 0x02;
        r >>= 2;
        m_value = r & 0x0FFF;          //0b00111111111111
        m_state |= (r & 0x3000) >> 10; //0b11000000000000
    }

    inline void setAgc(int16_t& r)
    {
        m_state |= r & 0x02;
        r >>= 2;
        m_agc = r & 0x3F; // 0b00000000111111
    }

    volatile int16_t m_value_last;
    volatile int16_t m_value;
    volatile uint8_t m_state;
    volatile uint8_t m_agc;
    volatile int32_t m_distance;
    volatile int32_t m_distance_last;
    volatile int16_t m_speed;
    volatile int16_t m_speed_last;
    volatile int16_t m_accel;
    volatile bool m_clr_err_req;
    uint8_t m_cs_pin;
};

typedef pin<portg, 0> enl_cs;
typedef pin<portg, 1> enr_cs;
encoder_t enc_left(0);
encoder_t enc_right(1);

class encoders_comm_t
{
public:
    encoders_comm_t()
    {
        m_transmitting = false;
        m_counter = 0;
        m_res_part = 0;
        m_enc = NULL;
        m_state = ST_NONE;
    }

    void queue_cmd(const uint16_t& cmd)
    {
        m_buff.push(cmd >> 8);
        m_buff.push(cmd & 0xFF);
    }

    void start_transmission()
    {
        if(m_transmitting)
            return;

        m_transmitting = true;
        m_enc->toggle_cs(false);
        SPDR = m_buff.top();
    }

    void start()
    {
        m_state = ST_SW_RESET1;
        switch_encoder();
        send_sw_reset();
    }

    void stop()
    {
        m_state = ST_NONE;
        enc_left.toggle_cs(true);
        enc_right.toggle_cs(true);
    }

    void handle_spi()
    {
        if(!m_transmitting)
            return;

        ++m_counter;
        if(m_counter & 0x01) // low byte
        {
            m_res_part = SPDR;
            SPDR = m_buff[m_counter];
        }
        else // high byte
        {
            if(m_counter != 2)
            {
                m_enc->cmd_result((m_buff[0] << 8 | m_buff[1]), (m_res_part << 8) | SPDR);
                m_buff.pop(2);
                m_counter -= 2;
            }

            if(m_counter < m_buff.size())
            {
                m_enc->toggle_cs(true);
                m_enc->toggle_cs(false);
                SPDR = m_buff[m_counter];
            }
            else
            {
                m_enc->toggle_cs(true);
                m_transmitting = false;
                m_buff.clear();
                m_counter = 0;
                trans_done();
            }
        }
    }

    void setEnc(encoder_t *enc) { m_enc = enc; }
    bool isTransmitting() const { return m_transmitting; }

private:
    void switch_encoder()
    {
        m_enc = (m_enc == &enc_left) ? &enc_right : &enc_left;
    }

    void send_read_seq()
    {
        queue_cmd(c_angular_data_register);
        queue_cmd(c_automatic_gain_control_register);
        // FIXME: this one should be NOP, but NOP causes
        // encoder to stop transmitting.
        queue_cmd(c_automatic_gain_control_register);
        start_transmission();
    }

    void send_sw_reset()
    {
        queue_cmd(c_sw_reset_cmd);
        queue_cmd(c_sw_reset_data);
        start_transmission();
    }

    void trans_done()
    {
        switch(m_state)
        {
            case ST_SW_RESET1:
                m_state = ST_SW_RESET2;
                switch_encoder();
                send_sw_reset();
                break;
            case ST_SW_RESET2:
                m_state = ST_READING;
                switch_encoder();
                send_read_seq();
                break;
            case ST_READING:
            {
                encoder_t *e = m_enc;

                switch_encoder();
                if(m_enc->takeErrReqStatus())
                    queue_cmd(c_clear_error_flag_register);
                send_read_seq();

                e->compute();
                break;
            }
        }
    }

    enum { ST_NONE, ST_SW_RESET1, ST_SW_RESET2, ST_READING };

    avrlib::buffer<uint8_t, 8> m_buff;
    volatile bool m_transmitting;
    volatile uint8_t m_counter;
    volatile uint8_t m_res_part;
    encoder_t *m_enc;
    volatile uint8_t m_state;
} encoders_comm;

#ifndef SPI_STC_vect
#define SPI_STC_vect SPI__STC_vect
#endif

ISR(SPI_STC_vect)
{
    encoders_comm.handle_spi();
}

typedef pin<portb, 3> MISO;
typedef pin<portb, 2> MOSI;
typedef pin<portb, 1> SCK;
typedef pin<portb, 0> SS;

void encoders_init()
{
    enl_cs::make_high();
    enr_cs::make_high();

    MISO::set(true);//pull-up
    MOSI::output(true);
    SCK::output(true);
    SS::set(true);//pull-up
    SPCR = (1<<SPIE) | (1<<SPE) | (1<<MSTR) | (1<<SPR1) | (1<<SPR0) | (1<<CPHA);

    encoders_comm.start();
}

#endif
