#ifndef _LIFE_H
#define _LIFE_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <avr/wdt.h> 
#include <avr/sfr_defs.h>
#include <avr/pgmspace.h>
#include <stdlib.h>

#define RED_DDR DDRB
#define GREEN_DDR DDRB
#define BLUE_DDR DDRB

#define RED_PORT PORTB
#define GREEN_PORT PORTB
#define BLUE_PORT PORTB

#define RED_PIN 1
#define GREEN_PIN 2
#define BLUE_PIN 0
#define OUTPUT_PIN 

#define LED_DDR DDRA
#define LED_PORT PORTA
#define LED_PIN 1

#define TOUCH_DDR DDRA
#define TOUCH_PORT PORTA
#define TOUCH_PIN 0
#define TOUCH_ADC 0

#define IN1_ADC 2
#define IN2_ADC 3
#define IN3_ADC 4
#define IN4_ADC 5

volatile static uint8_t red_level;
volatile static uint8_t green_level;
volatile static uint8_t blue_level;
volatile static uint8_t output_level;
volatile static uint8_t next_phase;

inline void led_on();
inline void led_off();

inline void timer_init();
inline void adc_init();
uint16_t cap_measure();
inline void adc_channel(uint8_t channel);
inline uint16_t adc_get();

#endif
