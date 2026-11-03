#ifndef __DHT_H
#define __DHT_H

#include <stdint.h>

// Wire ABI: keep these values in sync with klippy/extras/dht_sensor.py.
enum {
    DHT_OK, DHT_NO_RESPONSE, DHT_TIMEOUT, DHT_PULSE_ERROR,
    DHT_CHECKSUM, DHT_OUT_OF_RANGE, DHT_BUS_BUSY,
    DHT_PENDING = 255
};
enum { DHT11 = 11, DHT22 = 22 };
enum {
    DHT_WAIT_HIGH, DHT_WAIT_RESPONSE, DHT_RESPONSE_LOW,
    DHT_RESPONSE_HIGH, DHT_BIT_LOW, DHT_BIT_HIGH, DHT_COMPLETE
};

struct dht_decoder {
    uint16_t edge_clock;
    uint8_t state, bits, data[5];
};

// Pulse timestamps are 16MHz ticks. A scheduler watchdog bounds every gap.
void dht_decode_start(struct dht_decoder *d, uint8_t level, uint16_t clock);
uint8_t dht_decode_edge(struct dht_decoder *d, uint8_t level, uint16_t clock);
uint8_t dht_decode_frame(uint8_t type, const uint8_t *data,
                         int16_t *temp10, uint16_t *humidity10);

struct dht_gpio {
    uint8_t pin, pullup, level;
};
struct dht_gpio dht_gpio_setup(uint32_t pin, uint8_t pullup);
uint8_t dht_gpio_read(struct dht_gpio *g);
void dht_gpio_low(struct dht_gpio *g);
void dht_gpio_stop(struct dht_gpio *g);
void dht_gpio_receive(struct dht_gpio *g, struct dht_decoder *d);
uint16_t dht_gpio_clock(void);
void dht_edge(uint8_t level, uint16_t clock);

#endif
