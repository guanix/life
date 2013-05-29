#include "life.h"

int main()
{
    // initialize LED
    LED_DDR = _BV(LED_PIN);
    led_off();

    RED_DDR = _BV(RED_PIN);
    GREEN_DDR = _BV(GREEN_PIN);
    BLUE_DDR = _BV(BLUE_PIN);

    srand(1234);

    red_level = rand() >> 13;
    green_level = rand() >> 13;
    blue_level = rand() >> 13;
    output_level = rand() >> 13;

    timer_init();
    adc_init();
}

void led_on()
{
    LED_PORT |= _BV(LED_PIN);
}

void led_off()
{
    LED_PORT &= ~(_BV(LED_PIN));
}

void timer_init()
{
    TCCR0B = _BV(CS01) | _BV(CS00); // divide by 64 I think
    TCNT0 = 0;
    next_phase = 0;
    OCR0A = 1<<next_phase;
    TIMSK0 = _BV(OCIE0A) | _BV(TOIE0);
    sei();
}

void adc_init()
{
    // initialize the ADC
    ADMUX |= _BV(REFS0);
    ADCSRA |= _BV(ADPS2) | _BV(ADPS1);

    ADCSRA |= _BV(ADEN);
}

inline void adc_channel(uint8_t channel)
{
    ADMUX &= ~(0b11111);
    ADMUX |= 0b11111 & channel;
}

inline uint16_t adc_get()
{
    ADCSRA |= _BV(ADSC);
    while (!(ADCSRA & _BV(ADIF)));
    ADCSRA |= _BV(ADIF);
    return ADC;
}

uint16_t cap_measure()
{
    uint16_t retval;

    retval = 0;

}

ISR(TIM0_COMPA_vect)
{
    if (red_level & (1<<next_phase)) {
        RED_PORT |= _BV(RED_PIN);
    } else {
        RED_PORT &= ~(_BV(RED_PIN));
    }

    if (green_level & (1<<next_phase)) {
        GREEN_PORT |= _BV(GREEN_PIN);
    } else {
        GREEN_PORT &= ~(_BV(GREEN_PIN));
    }

    if (blue_level & (1<<next_phase)) {
        BLUE_PORT |= _BV(BLUE_PIN);
    } else {
        BLUE_PORT &= ~(_BV(BLUE_PIN));
    }

    next_phase++;
    if (next_phase == 3) {
        next_phase = 0;
    }
    OCR0A = 1<<next_phase;
    TCNT0 = 0;
}

EMPTY_INTERRUPT(TIM0_OVF_vect);
