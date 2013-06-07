#ifndef KVETINAC_BOOTSEQ_HPP
#define KVETINAC_BOOTSEQ_HPP

// This bootseq is the same as avrlib:bootseq, except it calls
// kvetinac_bootseq_reset() instead of bootseq_reset().
// We need to stop encoder's SPI protocol before stopping the chip.
// kvetinac_bootseq_reset() is defined in kvetinac.hpp because it
// needs encoder.hpp include
void kvetinac_bootseq_reset();

class kvetinac_bootseq
{
public:
    kvetinac_bootseq()
        : m_state(0)
    {
    }

    uint8_t check(uint8_t v)
    {
        static uint8_t const seq[] = { 0x74, 0x7E, 0x7A, 0x33 };

        if (seq[m_state++] != v)
            m_state = 0;

        if (m_state == 4)
            kvetinac_bootseq_reset();

        return v;
    }

private:
    uint8_t m_state;
};

#endif
