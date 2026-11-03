// Interrupt / open-drain GPIO backend for ATmega328P, 16MHz only.
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <avr/interrupt.h>
#include "autoconf.h"
#include "command.h"
#include "dht.h"
#include "internal.h"
#include "sched.h"

#if !CONFIG_MACH_atmega328p || CONFIG_CLOCK_FREQ != 16000000 \
    || CONFIG_HAVE_STEPPER || CONFIG_WANT_HARD_PWM
#error "DHT requires ATmega328P at 16MHz without steppers or hardware PWM"
#endif

static struct dht_gpio *receiver;

uint16_t
dht_gpio_clock(void)
{
    return TCNT1;
}

uint8_t
dht_gpio_read(struct dht_gpio *g)
{
    return g->pin == GPIO('B', 5) ? !!(PINB & (1<<5)) : !!(PIND & (1<<2));
}

// Caller holds the IRQ lock. Never drive the wire HIGH.
void
dht_gpio_stop(struct dht_gpio *g)
{
    receiver = NULL;
    if (g->pin == GPIO('B', 5)) {
        PCMSK0 &= ~(1<<PCINT5);
        if (!PCMSK0)
            PCICR &= ~(1<<PCIE0);
        DDRB &= ~(1<<5);
        if (g->pullup)
            PORTB |= 1<<5;
        else
            PORTB &= ~(1<<5);
    } else {
        EIMSK &= ~(1<<INT0);
        DDRD &= ~(1<<2);
        if (g->pullup)
            PORTD |= 1<<2;
        else
            PORTD &= ~(1<<2);
    }
}

struct dht_gpio
dht_gpio_setup(uint32_t pin, uint8_t pullup)
{
    if ((pin != GPIO('B', 5) && pin != GPIO('D', 2)) || pullup > 1)
        shutdown("DHT: only PB5 (D13) and PD2 (D2) are supported");
    if (pin == GPIO('B', 5)) {
        if (PCMSK0 || (PCICR & (1<<PCIE0)) || (SPCR & (1<<SPE)))
            shutdown("DHT: PB5 PCINT0/SPI conflict");
    } else if (EIMSK & (1<<INT0)) {
        shutdown("DHT: INT0 already in use");
    }
    struct dht_gpio g = { .pin = pin, .pullup = pullup };
    dht_gpio_stop(&g);
    return g;
}

void
dht_gpio_low(struct dht_gpio *g)
{
    dht_gpio_stop(g);
    if (g->pin == GPIO('B', 5)) {
        PORTB &= ~(1<<5);
        DDRB |= 1<<5;
    } else {
        PORTD &= ~(1<<2);
        DDRD |= 1<<2;
    }
}

void
dht_gpio_receive(struct dht_gpio *g, struct dht_decoder *d)
{
    dht_gpio_stop(g);
    // Clear stale flags BEFORE sampling/enabling. A late pull-up edge may
    // still arrive; WAIT_HIGH handles it without mistaking it for a response.
    if (g->pin == GPIO('B', 5))
        PCIFR = 1<<PCIF0;
    else {
        EICRA = (EICRA & ~((1<<ISC01) | (1<<ISC00))) | (1<<ISC00);
        EIFR = 1<<INTF0;
    }
    uint16_t clock = TCNT1;
    g->level = dht_gpio_read(g);
    dht_decode_start(d, g->level, clock);
    receiver = g;
    if (g->pin == GPIO('B', 5)) {
        PCMSK0 |= 1<<PCINT5;
        PCICR |= 1<<PCIE0;
    } else {
        EIMSK |= 1<<INT0;
    }
}

ISR(PCINT0_vect)
{
    uint16_t clock = TCNT1;
    uint8_t level = !!(PINB & (1<<5));
    struct dht_gpio *g = receiver;
    if (g && g->pin == GPIO('B', 5) && level != g->level) {
        g->level = level;
        dht_edge(level, clock);
    }
}

ISR(INT0_vect)
{
    uint16_t clock = TCNT1;
    uint8_t level = !!(PIND & (1<<2));
    struct dht_gpio *g = receiver;
    if (g && g->pin == GPIO('D', 2) && level != g->level) {
        g->level = level;
        dht_edge(level, clock);
    }
}
