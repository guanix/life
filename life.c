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

uint8_t cap_cal;

uint8_t state;

#define STATES 9

// R G B
const uint8_t palette[STATES][3] PROGMEM = {
    {0, 0, 0},
    {0, 0b01110000, 0b11110000},
    {0b11110000, 0, 0},
    {0b11110000, 0b01110000, 0},
    {0b11110000, 0b11110000, 0},
    {0, 0b11110000, 0},
    {0, 0b11110000, 0b11110000},
    {0, 0, 0b11110000},
    {0b11110000, 0, 0b11110000}
};

// Mapping of state to PWM value for OCR0B
const uint8_t palette_pwm[STATES] PROGMEM = {
    0,
    42,
    70,
    98,
    126,
    154,
    182,
    210,
    238
};

// Mapping of 8-bit truncated ADC value to state
const uint8_t palette_adc[STATES] PROGMEM = {
    28,
    56,
    84,
    112,
    140,
    168,
    196,
    224,
    245
};

// Mapping of rand value (0 to 0x7fff) to state
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

uint8_t adc_to_state(uint8_t a)
{
    // ADC is 10 bits, we want 8 valies which is 3 bits
    for (uint8_t i = 0; i < STATES; i++) {
        uint8_t t = pgm_read_byte(&(palette_adc[i]));
        if (a <= t) {
            return i;
        }
    }

    return STATES;
}

uint8_t read_neighbor()
{
    uint32_t reading = 0;
    for (uint16_t i = 0; i < NEIGHBOR_READINGS; i++) {
        reading += adc_get_raw();
    }

    reading /= NEIGHBOR_READINGS;
    reading /= 4;
    return reading;
}

inline void read_neighbors()
{
    adc_channel(IN1_ADC);
    neighbors[0] = adc_to_state(read_neighbor());

    adc_channel(IN2_ADC);
    neighbors[1] = adc_to_state(read_neighbor());

    adc_channel(IN3_ADC);
    neighbors[2] = adc_to_state(read_neighbor());

    adc_channel(IN4_ADC);
    neighbors[3] = adc_to_state(read_neighbor());
}

void update_colors()
{
    red_level = pgm_read_byte(&(palette[state][0]));
    green_level = pgm_read_byte(&(palette[state][1]));
    blue_level = pgm_read_byte(&(palette[state][2]));

    OCR0B = pgm_read_byte(&(palette_pwm[state]));
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

    OUTPUT_DDR |= _BV(OUTPUT_PIN);
    // Set outputs

    RED_DDR |= _BV(RED_PIN);
    GREEN_DDR |= _BV(GREEN_PIN);
    BLUE_DDR |= _BV(BLUE_PIN);

    OUTPUT_DDR |= _BV(OUTPUT_PIN);

    timer_init();
    adc_init();
//    srand(1234);
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

    uint8_t count = 0, count_s = 0;

    // main event loop
    while (1) {
        if (touch_measure() < cap_cal) {
            led_on();
            state = rand_to_state(rand());
//            state++;
//            if (state == STATES) state = 0;
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
            if (rand() < RAND_MAX>>9) {
                blink(4);
                state = rand_to_state(rand());
                update_colors();
                continue;
            }

            // read the neighbors
            read_neighbors();
            for (uint8_t i = 0; i < NEIGHBORS; i++) {
                if (neighbors[i] != STATES &&
                        (neighbors[i] == state + 1 ||
                        (state == STATES && neighbors[i] == 0))) {
                    blink(2);
                    state = neighbors[i];
                    update_colors();
                    _delay_ms(2000);
                }
            }

            led_off();

            // recalibrate touch sensor every to seconds
            if (count_s++ == 20) {
                count_s = 0;
                touch_calibrate();
            }
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

void timer_init()
{
    // set up timer 1 for LEDs, /64 prescaling
    TCCR1B = _BV(CS11) | _BV(CS10);
    next_phase = 0;
    OCR1A = 1<<next_phase;
    TIMSK1 = _BV(OCIE1A);

    TCNT1 = 0;

    // set up timer 0 for analog output, /8 prescaling
    // fast PWM mode
    TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
    TCCR0B = _BV(CS01);
    // start out at zero
    OCR0B = 0;
    TCNT0 = 0;

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

inline void seed()
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

ISR(TIM1_COMPA_vect)
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

    // advance to next phase, update OCR, reset timer
    next_phase++;
    if (next_phase == PWM_BITS) {
        // ignore first couple of phases because they're too short
        next_phase = 3;
    }
    OCR1A = 1<<next_phase;
    TCNT1 = 0;
}
