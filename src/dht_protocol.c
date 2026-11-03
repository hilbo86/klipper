// DHT11 / AM2302 (DHT22) frame decoder, independent of MCU and scheduler.
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h>
#include "dht.h"

void
dht_decode_start(struct dht_decoder *d, uint8_t level, uint16_t clock)
{
    memset(d->data, 0, sizeof(d->data));
    d->bits = 0;
    d->edge_clock = clock;
    d->state = level ? DHT_WAIT_RESPONSE : DHT_WAIT_HIGH;
}

uint8_t
dht_decode_edge(struct dht_decoder *d, uint8_t level, uint16_t clock)
{
    uint16_t ticks = clock - d->edge_clock;
    switch (d->state) {
    case DHT_WAIT_HIGH:
        // The pull-up's rising edge is ours, not the sensor response.
        if (!level || ticks > 50 * 16)
            return DHT_PULSE_ERROR;
        d->state = DHT_WAIT_RESPONSE;
        // Keep the timestamp of bus release for the response delay check.
        return DHT_PENDING;
    case DHT_WAIT_RESPONSE:
        if (level || ticks < 10 * 16 || ticks > 160 * 16)
            return DHT_PULSE_ERROR;
        d->state = DHT_RESPONSE_LOW;
        break;
    case DHT_RESPONSE_LOW:
    case DHT_RESPONSE_HIGH:
        if (level != (d->state == DHT_RESPONSE_LOW)
            || ticks < 45 * 16 || ticks > 120 * 16)
            return DHT_PULSE_ERROR;
        d->state++;
        break;
    case DHT_BIT_LOW:
        if (!level || ticks < 25 * 16 || ticks > 85 * 16)
            return DHT_PULSE_ERROR;
        d->state = DHT_BIT_HIGH;
        break;
    case DHT_BIT_HIGH: {
        if (level || ticks < 10 * 16 || ticks > 100 * 16)
            return DHT_PULSE_ERROR;
        uint8_t pos = d->bits / 8;
        d->data[pos] = (d->data[pos] << 1) | (ticks >= 50 * 16);
        if (++d->bits == 40) {
            d->state = DHT_COMPLETE;
            return DHT_OK;
        }
        d->state = DHT_BIT_LOW;
        break;
    }
    default:
        return DHT_PULSE_ERROR;
    }
    d->edge_clock = clock;
    return DHT_PENDING;
}

uint8_t
dht_decode_frame(uint8_t type, const uint8_t *b,
                 int16_t *temp10, uint16_t *humidity10)
{
    if ((uint8_t)(b[0] + b[1] + b[2] + b[3]) != b[4])
        return DHT_CHECKSUM;
    int16_t temp;
    uint16_t humidity;
    if (type == DHT11) {
        // Classic DHT11: decimal bytes are decimal tenths, not binary words.
        // Variants with sign bits / different decimal encodings are rejected.
        if (b[1] > 9 || b[3] > 9)
            return DHT_OUT_OF_RANGE;
        humidity = b[0] * 10 + b[1];
        temp = b[2] * 10 + b[3];
        if (humidity < 200 || humidity > 900 || temp > 500)
            return DHT_OUT_OF_RANGE;
    } else if (type == DHT22) {
        humidity = ((uint16_t)b[0] << 8) | b[1];
        temp = ((uint16_t)(b[2] & 0x7f) << 8) | b[3];
        if (b[2] & 0x80)
            temp = -temp;
        if (humidity > 1000 || temp < -400 || temp > 800)
            return DHT_OUT_OF_RANGE;
    } else {
        return DHT_OUT_OF_RANGE;
    }
    *temp10 = temp;
    *humidity10 = humidity;
    return DHT_OK;
}
