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

uint8_t red_level;
uint8_t green_level;
uint8_t blue_level;
uint8_t output_level;
uint8_t next_phase;

uint16_t cap_cal;

uint8_t state;

uint32_t time = 0;

#define STATES 9

// R G B
const uint8_t palette[STATES][3] PROGMEM = {
    {0, 0, 0},
    {128, 0, 0},
    {255, 0, 0},
    {255, 165, 0},
    {255, 255, 0},
    {0, 255, 0},
    {0, 255, 255},
    {0, 0, 255},
    {255, 0, 255}
};

const uint8_t palette_pwm[STATES] PROGMEM = {
    14,
    42,
    70,
    98,
    126,
    154,
    182,
    210,
    238
};

const uint16_t palette_adc[STATES-1] PROGMEM = {
    114,
    228,
    342,
    456,
    570,
    684,
    798,
    912
};

const int palette_rand[STATES-1] PROGMEM = {
    3640,
    7280,
    10920,
    14560,
    18200,
    21840,
    25480,
    29120
};

uint8_t neighbors[NEIGHBORS];

uint8_t rand_to_state(int r)
{
    // rand_max is 15 bits, we want 8 values which is 3 bits
    for (uint8_t i = 0; i < STATES-1; i++) {
        int t = pgm_read_word(&(palette_rand[i]));
        if (r <= t) {
            return i;
        }
    }

    return STATES-1;
}

uint8_t adc_to_state(uint16_t a)
{
    // ADC is 10 bits, we want 8 valies which is 3 bits
    for (uint8_t i = 0; i < STATES-1; i++) {
        uint16_t t = pgm_read_word(&(palette_adc[i]));
        if (a <= t) {
            return i;
        }
    }

    return STATES-1;
}

void read_neighbors()
{
    adc_channel(IN1_ADC);
    neighbors[0] = adc_to_state(adc_get());

    adc_channel(IN2_ADC);
    neighbors[1] = adc_to_state(adc_get());

    adc_channel(IN3_ADC);
    neighbors[2] = adc_to_state(adc_get());

    adc_channel(IN4_ADC);
    neighbors[3] = adc_to_state(adc_get());
}

void update_colors()
{
    red_level = pgm_read_byte(&(palette[state][0]));
    green_level = pgm_read_byte(&(palette[state][1]));
    blue_level = pgm_read_byte(&(palette[state][2]));

    output_level = pgm_read_byte(&(palette_pwm[state]));
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

    RED_DDR |= _BV(RED_PIN);
    GREEN_DDR |= _BV(GREEN_PIN);
    BLUE_DDR |= _BV(BLUE_PIN);

    OUTPUT_DDR |= _BV(OUTPUT_PIN);

    timer_init();
    adc_init();
    seed();

    touch_calibrate();

    // cycle through the colors
    for (state = 0; state < STATES; state++) {
        update_colors();
        _delay_ms(500);
    }

    // sleep for a random amount of time up to 1 second
    for (uint8_t i = 0; i < rand() >> 8; i++) {
        _delay_ms(10);
    }

    // initial state
    state = rand_to_state(rand());
    update_colors();

    uint32_t last_update = time;
    uint32_t last_calibrate = time;
    uint32_t diff_update, diff_calibrate;

    // main event loop
    while (1) {
        if (touch_measure() < cap_cal) {
            led_on();
            state = rand_to_state(rand());
            update_colors();
            // wait till finger lifted
            _delay_ms(500);
            while (touch_measure() < cap_cal);
            led_off();
            _delay_ms(2000);
        }

        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            diff_calibrate = time - last_calibrate;
        }

        // recalibrate touch every 20 seconds
        if (diff_calibrate > 20000) {
            _delay_ms(50);
            if (touch_measure() >= cap_cal) {
                // 10 seconds since last update
                touch_calibrate();
                ATOMIC_BLOCK(ATOMIC_FORCEON) {
                    last_calibrate = time;
                }
            }
        
            _delay_ms(2000);
            continue;
        }

        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            diff_update = time - last_update;
        }

        // read neighbors and advance state machine
        if (diff_update > 800) {
            // Randomly change colors instead
            if (rand() < 150) {
                blink(4);
                state = rand_to_state(rand());
                update_colors();
                continue;
            }

            // read the neighbors
            read_neighbors();
            for (uint8_t i = 0; i < NEIGHBORS; i++) {
                if (neighbors[i] == state + 1 ||
                        (state == STATES && neighbors[i] == 0)) {
                    blink(2);
                    state = neighbors[i];
                    update_colors();
                    _delay_ms(2000);
                }
            }

            ATOMIC_BLOCK(ATOMIC_FORCEON) {
                last_update = time;
            }
            continue;
        }

        _delay_ms(25);
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

void timer_init()
{
    // set up timer 0
    TCCR0B = _BV(CS01) | _BV(CS00); // divide by 64 I think
    next_phase = 0;
    OCR0A = 1<<next_phase;
    TIMSK0 = _BV(OCIE0A);

    // start timer1
    TCCR1B = _BV(CS10); // no prescaling
    OCR1A = 8000;
    TIMSK1 = _BV(OCIE1A);

    TCNT0 = 0;
    TCNT1 = 0;

    sei();
}

void adc_init()
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

inline uint16_t adc_get()
{
    ADCSRA |= _BV(ADSC);
    while (!(ADCSRA & _BV(ADIF)));
    ADCSRA |= _BV(ADIF);
    return ADC;
}

void touch_calibrate()
{
    // we will do 16 tests 100 ms apart
    blink(3);

    uint16_t cal = 0;

    for (uint8_t i = 0; i < TOUCH_CALS; i++) {
        cal += touch_measure_one();
        _delay_ms(100);
    }

    // take the average
    cal /= TOUCH_CALS;
    cap_cal = cal - 150;
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

    for (uint8_t i = 0; i < 4; i++) {
        retval += touch_measure_one();
    }

    return retval/4;
}

ISR(TIM0_COMPA_vect)
{
    // for each color (and analog output), set output according to the
    // binary code modulation
    if (red_level & _BV(next_phase)) {
        RED_PORT |= _BV(RED_PIN);
    } else {
        RED_PORT &= ~(_BV(RED_PIN));
    }

    if (green_level & _BV(next_phase)) {
        GREEN_PORT |= _BV(GREEN_PIN);
    } else {
        GREEN_PORT &= ~(_BV(GREEN_PIN));
    }

    if (blue_level & _BV(next_phase)) {
        BLUE_PORT |= _BV(BLUE_PIN);
    } else {
        BLUE_PORT &= ~(_BV(BLUE_PIN));
    }

    if (blue_level & _BV(next_phase)) {
        BLUE_PORT |= _BV(BLUE_PIN);
    } else {
        BLUE_PORT &= ~(_BV(BLUE_PIN));
    }

    if (output_level & _BV(next_phase)) {
        OUTPUT_PORT |= _BV(OUTPUT_PIN);
    } else {
        OUTPUT_PORT &= ~(_BV(OUTPUT_PIN));
    }

    // advance to next phase, update OCR, reset timer
    next_phase++;
    if (next_phase == PWM_BITS) {
        // ignore phase 1 and 2 because they're too short
        next_phase = 2;
    }
    OCR0A = 1<<next_phase;
    TCNT0 = 0;
}

// this updates our 1ms counter for global clock
// 32 bits, good for 50 days
ISR(TIM1_COMPA_vect) {
    time++;
}
