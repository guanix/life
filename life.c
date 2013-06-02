#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/wdt.h> 
#include <avr/sfr_defs.h>
#include <avr/pgmspace.h>
#include <stdlib.h>

#include "life.h"

uint8_t rgb_levels[3] = {0, 0, 0};
uint8_t output_level;
uint8_t next_phase;

uint8_t cap_cal;

// start in state zero (off)
uint8_t state = 0;
uint8_t output = 0;

// clocks
uint16_t clock = 0;

#define STATES 9

// R G B
const uint8_t palette[STATES][3] PROGMEM = {
    {0, 0, 0},
    {0, 165, 255},
    {255, 0, 0},
    {255, 165, 0},
    {255, 255, 0},
    {0, 255, 0},
    {0, 255, 255},
    {0, 0, 255},
    {255, 0, 255}
};

const uint8_t palette_rand[STATES-1] PROGMEM = {
    14,
    28,
    42,
    56,
    70,
    84,
    98,
    112
};

// last known neighbor state
uint8_t neighbors[NEIGHBORS];
uint16_t neighbors_last_pc[NEIGHBORS];

uint8_t rand_to_state(int r)
{
    uint8_t r2 = r >> 8;

    // rand_max is 15 bits, we want 8 values which is 3 bits
    for (uint8_t i = 0; i < STATES-1; i++) {
        int t = pgm_read_word(&(palette_rand[i]));
        if (r2 <= t) {
            return i;
        }
    }

    return STATES-1;
}

void update_colors()
{
    for (uint8_t i = 0; i < 3; i++) {
        rgb_levels[i] = pgm_read_byte(&(palette[state][i]));
    }
}

void blink(uint8_t times)
{
    for (uint8_t i = 0; i < times; i++) {
        led_on();
        _delay_ms(120);
        led_off();
        _delay_ms(120);
    }
}

int main()
{
    // initialize LED
    LED_DDR |= _BV(LED_PIN);
    led_off();

    // Wakeup inducation
    blink(2);

    // Set outputs

    RGB_DDR |= _BV(RED_PIN) | _BV(GREEN_PIN) | _BV(BLUE_PIN);

    OUTPUT_PORT |= _BV(OUTPUT_PIN);
    OUTPUT_DDR |= _BV(OUTPUT_PIN);

    // pullups on all the inputs
    for (uint8_t i = 0; i < IN_PINS; i++) {
        IN_DDR &= ~(_BV(in_pins[i]));
        IN_PORT |= _BV(in_pins[i]);
    }

    timer_init();
    adc_init();
//    srand(1234);
    seed();

    // cycle through the colors
    // disabled to save space
#if 1
    for (state = 0; state < STATES; state++) {
        update_colors();
        _delay_ms(500);
    }
#endif

    // sleep for a random amount of time up to 1 second
    for (uint8_t i = 0; i < rand() >> 8; i++) {
        _delay_ms(10);
    }

    touch_calibrate();

    // initial state
    state = rand_to_state(rand());
    update_colors();

    uint8_t count = 0;

    // main event loop
    while (1) {
        if (touch_measure() < cap_cal) {
            led_on();
//            state = rand_to_state(rand());
            state++;
            if (state == STATES) state = 0;
            update_colors();
            // wait till finger lifted
            _delay_ms(1000);
            led_off();
        }

        // read neighbors and advance state machine
        if (count >= 1000/LOOP_INTERVAL) {
            count = 0;
            led_on();

            // Randomly change colors instead
            if (rand() < RAND_MAX>>12) {
                blink(4);
                state = rand_to_state(rand());
                update_colors();
                continue;
            }

            // read the neighbors
            for (uint8_t i = 0; i < NEIGHBORS; i++) {
                if (neighbors[i] == state + 1 ||
                        (state == STATES && neighbors[i] == 0)) {
                    blink(2);
                    state = neighbors[i];
                    update_colors();
                    _delay_ms(2000);
                }
            }

            led_off();
        }

        _delay_ms(LOOP_INTERVAL);
        count++;
    }
}

void led_on()
{
    LED_PORT |= _BV(LED_PIN);
}

void led_off()
{
    LED_PORT &= ~(_BV(LED_PIN));
}

inline void timer_init()
{
    // set up timer 1 for LEDs, /64 prescaling
    TCCR0B = _BV(CS01) | _BV(CS00);
    next_phase = 0;
    OCR0A = 1<<next_phase;
    TIMSK0 = _BV(OCIE1A);

    TCNT0 = 0;

    TCCR1B = _BV(CS11); // /8 prescaling so TCNT1 counts in microseconds

    // OCR1A is used for our own output
    // output starts at 0
    // we send out every 10000 microseconds
    output = 0;
    OCR1A = 10000;
    TIMSK1 = _BV(OCIE1A) | _BV(TOIE1);

    // enable pin change interrupts on inputs
    GIFR |= _BV(PCIF0);
    for (uint8_t i = 0; i < IN_PINS; i++) {
        PCMSK0 |= _BV(in_pins[i]);
    }

    sei();
}

inline void adc_init()
{
    // initialize the ADC
    ADCSRA |= _BV(ADPS2) | _BV(ADPS1);
    ADCSRA |= _BV(ADEN);
}

void adc_channel(uint8_t channel)
{
    ADMUX &= ~(0b11111);
    ADMUX |= 0b11111 & channel;
}

uint8_t adc_get()
{
    ADCSRA |= _BV(ADSC);
    while (!(ADCSRA & _BV(ADIF)));
    ADCSRA |= _BV(ADIF);
    return ADC / 4;
}

uint16_t adc_get_raw()
{
    ADCSRA |= _BV(ADSC);
    while (!(ADCSRA & _BV(ADIF)));
    ADCSRA |= _BV(ADIF);
    return ADC;
}

inline void touch_calibrate()
{
    // hard code for now
    // we will do 16 tests 100 ms apart
    blink(3);

    // we will do the average of the three smallest and subtract threshold

    uint16_t cal = 0;

    cli();

    for (uint8_t i = 0; i < TOUCH_CALS; i++) {
        cal += touch_measure_one();
        _delay_ms(TOUCH_CAL_INTERVAL);
    }

    sei();

    cap_cal = cal / TOUCH_CALS - TOUCH_THRESHOLD;
    if (cap_cal < 180) {
        cap_cal = 180;
    }
}

void seed()
{
    // generate 16 bits of entropy and seed the PRNG
    unsigned int seed = 0;

    adc_channel(TEMP_ADC);

    for (uint8_t i = 0; i < sizeof(unsigned int)*8; i++) {
        // shift left and set the LSB
        seed <<= 1;
        seed |= (adc_get() & 1);
    }

    srand(seed);
}

uint16_t touch_measure_one()
{
    // Charge the sensor cap with pullup
    TOUCH_PORT |= _BV(TOUCH_PIN);
    _delay_ms(1);
    TOUCH_PORT &= ~(_BV(TOUCH_PIN));

    // Discharge the ADC cap
    adc_channel(0b11111);
    _delay_us(500);
    adc_get();

    // Read ADC
    adc_channel(TOUCH_ADC);
    return adc_get();
}

uint16_t touch_measure()
{
    // average of 4 measurements
    uint16_t retval = 0;

    cli();

    for (uint8_t i = 0; i < TOUCH_MEASURES; i++) {
        retval += touch_measure_one();
    }
    
    sei();

    return retval/TOUCH_MEASURES;
}

ISR(TIM0_COMPA_vect)
{
    // for each color (and analog output), set output according to the
    // binary code modulation
    // this method wastes some RAM but saves us some program space
    for (uint8_t i = 0; i < 3; i++) {
        if (rgb_levels[i] & _BV(next_phase)) {
            RGB_PORT |= _BV(rgb_pins[i]);
        } else {
            RGB_PORT &= ~(_BV(rgb_pins[i]));
        }
    }

    // advance to next phase, update OCR, reset timer
    next_phase++;
    if (next_phase == PWM_BITS) {
        // ignore first couple of phases because they're too short
        next_phase = 1;
    }
    OCR0A = 1<<next_phase;
    TCNT0 = 0;
}

ISR(TIM1_COMPA_vect)
{
    // flip our output off
    if (output) {
        #if (OUTPUT_INTERVAL > (1<<15))
            #error OUTPUT_INTERVAL too large to be practical
        #endif
        OUTPUT_PORT &= ~(_BV(OUTPUT_PIN));
        OCR1A += OUTPUT_INTERVAL;
        output = 0;
    } else {
        // flip output on
        OUTPUT_PORT |= _BV(OUTPUT_PIN);
        OCR1A += (state+1)*100;
        output = 1;
    }
}

ISR(TIM1_OVF_vect)
{
    // update timer
    TCNT1 = 0;
    clock++;
}

ISR(PCINT0_vect)
{
    for (uint8_t i = 0; i < IN_PINS; i++) {
        // high to low
        if (neighbors_last_pc[i] == 0 && !(IN_PIN & _BV(in_pins[i]))) {
            neighbors_last_pc[i] = TCNT1;
        } else if (neighbors_last_pc[i] > 0 && (IN_PIN & _BV(in_pins[i]))) {
            // low to high
            uint16_t period = TCNT1 - neighbors_last_pc[i];
            neighbors_last_pc[i] = 0;

            // too high;
            if (period > 910) continue;

            period += 10;

            // we have to be within 10% of the correct period
            uint8_t modulus = period % 100;
            if (modulus <= 20 && period >= 1 && period <= 9) {
                neighbors[i] = (period - modulus)/100;
            }
        }
    }
}
