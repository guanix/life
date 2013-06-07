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

uint8_t next_phase;
uint8_t cap_cal;
uint8_t state;

#define STATES 8

// R G B
const uint8_t palette[STATES][3] PROGMEM = {
    {0, 0, 0},
    {1, 0, 0}, // red
    {1, 1, 0}, // yellow
    {0, 1, 0}, // green
    {0, 1, 1}, // cyan
    {0, 0, 1}, // blue
    {1, 0, 1}, // pink
    {1, 1, 1} // white
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
    210
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
    224
};

// Mapping of rand value (0 to 0x7fff) to state
const int palette_rand[STATES-1] PROGMEM = {
    4100,
    8200,
    12300,
    16400,
    20500,
    24600,
    28700
};

uint8_t neighbors[NEIGHBORS];

// temperature samples ring buffer
uint8_t temp_samples[TEMP_SAMPLES];
uint8_t temp_top = TEMP_SAMPLES - 1;

// store the freshest temperature sample, return the oldest one
uint8_t temp_getput(uint8_t val)
{
    uint8_t temp_bot = temp_top + 1;
    if (temp_top == TEMP_SAMPLES) {
        temp_bot = 0;
    }

    uint8_t retval = temp_samples[temp_bot];
    temp_samples[temp_bot] = val;
    temp_top = temp_bot;
    return retval;
}

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

void advance_state()
{
    state++;
    if (state == STATES) {
        state = 0;
    }
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
    if (pgm_read_byte(&(palette[state][0]))) {
        RGB_PORT |= _BV(RED_PIN);
    } else {
        RGB_PORT &= ~(_BV(RED_PIN));
    }

    if (pgm_read_byte(&(palette[state][1]))) {
        RGB_PORT |= _BV(GREEN_PIN);
    } else {
        RGB_PORT &= ~(_BV(GREEN_PIN));
    }

    if (pgm_read_byte(&(palette[state][2]))) {
        RGB_PORT |= _BV(BLUE_PIN);
    } else {
        RGB_PORT &= ~(_BV(BLUE_PIN));
    }

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

    RGB_DDR |= _BV(RED_PIN) | _BV(GREEN_PIN) | _BV(BLUE_PIN);

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
            advance_state();
            update_colors();
            // wait till finger lifted
            _delay_ms(1000);
            led_off();
        }

        // read neighbors and advance state machine
#if (LOOP_INTERVAL<=3)
    #error LOOP_INTERVAL too small
#endif
        
        if (count >= 1000/LOOP_INTERVAL) {
            count = 0;
            blink(1);

            // do a temperature measurement
            uint8_t temp = temp_measure();
            uint8_t old_temp = temp_getput(temp);
            if (old_temp > 0 && temp - old_temp > 3) {
                blink(5);
                advance_state();
                update_colors();
                continue;
            }

            // Randomly change colors instead
            if (rand() < RAND_MAX>>9) {
                blink(4);
                advance_state();
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

            // recalibrate touch sensor every 200 seconds
            if (count_s++ == 200) {
                count_s = 0;
                if (touch_measure() >= cap_cal) {
                    _delay_ms(50);
                    if (touch_measure() >= cap_cal) {
                        touch_calibrate();
                    }
                }
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
    // set up timer 0 for analog output, /8 prescaling
    // fast PWM mode
    TCCR0A = _BV(COM0B1) | _BV(WGM01) | _BV(WGM00);
    TCCR0B = _BV(CS01);
    // start out at zero
    OCR0B = 0;
    TCNT0 = 0;
}

void adc_init()
{
    // initialize the ADC
    ADCSRA |= _BV(ADPS2) | _BV(ADPS1);
    ADCSRA |= _BV(ADEN);
}

void adc_channel(uint8_t channel)
{
    ADMUX &= ~(0b111111);
    ADMUX |= 0b111111 & channel;
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

//    cli();

    for (uint8_t i = 0; i < TOUCH_CALS; i++) {
        cal += touch_measure_one();
        _delay_ms(TOUCH_CAL_INTERVAL);
    }

//    sei();

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
    _delay_us(500);

    // Discharge the ADC cap
    adc_channel(0b11111);
    _delay_us(500);
    adc_get();
    _delay_us(500);

    // Read ADC
    adc_channel(TOUCH_ADC);
    return adc_get();
}

uint8_t temp_measure()
{
    uint16_t retval = 0;

    for (uint8_t i = 0; i < TOUCH_MEASURES; i++) {
        adc_channel(TEMP_ADC);
        retval += adc_get_raw();
    }

    retval /= TOUCH_MEASURES;
    retval /= 4;
    return retval;
}

uint16_t touch_measure()
{
    // average of 4 measurements
    uint16_t retval = 0;

//    cli();

    for (uint8_t i = 0; i < TOUCH_MEASURES; i++) {
        retval += touch_measure_one();
    }
    
//    sei();

    return retval/TOUCH_MEASURES;
}
