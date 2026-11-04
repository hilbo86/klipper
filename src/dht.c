// Nonblocking DHT sampling on auxiliary MCUs (no heaters / steppers).
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h>
#include "basecmd.h"
#include "board/irq.h"
#include "board/misc.h"
#include "command.h"
#include "dht.h"
#include "sched.h"

enum { DHT_STOPPED, DHT_START, DHT_RELEASE, DHT_RECEIVE, DHT_REPORT };
struct dht {
    struct timer timer;
    uint32_t rest_ticks, start_clock, next_start, ready_clock;
    struct dht_decoder decoder;
    struct dht_gpio gpio;
    uint8_t type, phase, pending, status;
};
static struct task_wake dht_wake;

DECL_CONSTANT("DHT_VERSION", 1);

static void
dht_finish(struct dht *d, uint8_t status)
{
    dht_gpio_stop(&d->gpio);
    d->status = status;
    d->phase = DHT_REPORT;
    d->pending = 1;
    sched_wake_task(&dht_wake);
}

// Called only for the armed GPIO receiver, with IRQs disabled.
void
dht_edge(struct dht_gpio *gpio, uint8_t level, uint16_t clock)
{
    struct dht *d = container_of(gpio, struct dht, gpio);
    uint8_t status = dht_decode_edge(&d->decoder, level, clock);
    if (status != DHT_PENDING)
        dht_finish(d, status);
}

static uint_fast8_t
dht_event(struct timer *timer)
{
    struct dht *d = container_of(timer, struct dht, timer);
    switch (d->phase) {
    case DHT_START:
        // Never overwrite an unconsumed result, even with a starved task.
        if (d->pending) {
            timer->waketime += d->rest_ticks;
            return SF_RESCHEDULE;
        }
        d->next_start = timer->waketime + d->rest_ticks;
        d->start_clock = timer_read_time();
        d->ready_clock = d->start_clock
            + timer_from_us(d->type == DHT11 ? 1000001 : 2000001);
        if (!dht_gpio_read(&d->gpio)) {
            dht_finish(d, DHT_BUS_BUSY);
            break;
        }
        dht_gpio_low(&d->gpio);
        d->phase = DHT_RELEASE;
        timer->waketime = d->start_clock
            + timer_from_us(d->type == DHT11 ? 20000 : 2000);
        return SF_RESCHEDULE;
    case DHT_RELEASE: {
        // Prepare the scheduler before releasing DATA, leaving as little
        // masked time as possible before the sensor's first response edge.
        uint32_t clock = timer_read_time();
        d->phase = DHT_RECEIVE;
        timer->waketime = clock + timer_from_us(1000);
        dht_gpio_receive(&d->gpio, &d->decoder);
        return SF_RESCHEDULE;
    }
    case DHT_RECEIVE: {
        // A 1ms guard catches gaps over 200us before the 4096us TCNT1
        // wrap. Valid pulse sequences finish within 7800us by construction.
        uint16_t gap = dht_gpio_clock() - d->decoder.edge_clock;
        if (gap > 200 * 16) {
            uint8_t status = d->decoder.state <= DHT_WAIT_RESPONSE
                ? DHT_NO_RESPONSE : DHT_TIMEOUT;
            dht_finish(d, status);
            break;
        }
        timer->waketime += timer_from_us(1000);
        return SF_RESCHEDULE;
    }
    case DHT_REPORT:
        break;
    default:
        return SF_DONE;
    }
    d->phase = DHT_START;
    timer->waketime = d->next_start;
    return SF_RESCHEDULE;
}

void
command_config_dht(uint32_t *args)
{
    uint8_t oid;
    struct dht *other;
    foreach_oid(oid, other, command_config_dht) {
        (void)other;
        shutdown("DHT: only one sensor per MCU is supported");
    }
    if (args[2] != DHT11 && args[2] != DHT22)
        shutdown("DHT: invalid sensor type");
    // Validate before allocating an OID. A failed setup must not leave a
    // zero-initialized pin that the shutdown hook could mistake for PD2.
    irqstatus_t flag = irq_save();
    struct dht_gpio gpio = dht_gpio_setup(args[1], args[3]);
    irq_restore(flag);
    // oid_alloc uses Klipper's startup arena; no per-frame allocation.
    struct dht *d = oid_alloc(args[0], command_config_dht, sizeof(*d));
    d->gpio = gpio;
    d->type = args[2];
    d->timer.func = dht_event;
    d->ready_clock = timer_read_time() + timer_from_us(2000000);
}
DECL_COMMAND(command_config_dht,
             "config_dht oid=%c pin=%u sensor_type=%c pull_up=%c");

void
command_query_dht(uint32_t *args)
{
    struct dht *d = oid_lookup(args[0], command_config_dht);
    uint32_t rest = args[2];
    if (rest && (rest <= timer_from_us(d->type == DHT11 ? 1000000 : 2000000)
                 || rest > 0x7fffffff))
        shutdown("DHT: invalid sampling interval");
    irqstatus_t flag = irq_save();
    sched_del_timer(&d->timer);
    dht_gpio_stop(&d->gpio);
    d->phase = DHT_STOPPED;
    d->pending = 0;
    if (rest) {
        uint32_t clock = args[1], earliest = timer_read_time()
            + timer_from_us(1000);
        // A stopped sensor may be restarted after more than half a 32-bit
        // clock cycle. Only treat a bounded remaining warm-up as future.
        if ((uint32_t)(d->ready_clock - earliest) <= timer_from_us(2000001))
            earliest = d->ready_clock;
        if (timer_is_before(clock, earliest))
            clock = earliest;
        d->rest_ticks = rest;
        d->timer.waketime = clock;
        d->phase = DHT_START;
        sched_add_timer(&d->timer);
    }
    irq_restore(flag);
}
DECL_COMMAND(command_query_dht, "query_dht oid=%c clock=%u rest_ticks=%u");

void
dht_task(void)
{
    if (!sched_check_wake(&dht_wake))
        return;
    uint8_t oid;
    struct dht *d;
    foreach_oid(oid, d, command_config_dht) {
        irqstatus_t flag = irq_save();
        if (!d->pending) {
            irq_restore(flag);
            continue;
        }
        uint8_t status = d->status, data[5];
        uint32_t clock = d->start_clock;
        memcpy(data, d->decoder.data, sizeof(data));
        d->pending = 0;
        irq_restore(flag);
        int16_t temp10 = 0;
        uint16_t humidity10 = 0;
        if (status == DHT_OK)
            status = dht_decode_frame(d->type, data, &temp10, &humidity10);
        sendf("dht_result oid=%c clock=%u status=%c temp10=%hi humidity10=%hu",
              oid, clock, status, temp10, humidity10);
    }
}
DECL_TASK(dht_task);

void
dht_shutdown(void)
{
    uint8_t oid;
    struct dht *d;
    foreach_oid(oid, d, command_config_dht) {
        dht_gpio_stop(&d->gpio);
        d->phase = DHT_STOPPED;
        d->pending = 0;
    }
}
DECL_SHUTDOWN(dht_shutdown);
