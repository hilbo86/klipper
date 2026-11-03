// SAM3/SAM4/SAM E70 I2C Port
//
// Copyright (C) 2018  Florian Heilmann <Florian.Heilmann@gmx.net>
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "board/misc.h" // timer_from_us
#include "command.h" // shutdown
#include "gpio.h" // i2c_setup
#include "internal.h" // gpio_peripheral
#include "sched.h" // sched_shutdown
#include "i2ccmds.h" // I2C_BUS_SUCCESS

DECL_CONSTANT("I2C_BUS_RECOVERY", CONFIG_ATSAM_I2C_RECOVERY);

#if CONFIG_MACH_SAME70
#include "same70_i2c.h" // Fixes for upstream header changes
#endif

struct twi_info {
    void *dev;
    uint32_t dev_id;
    uint8_t scl_pin, sda_pin, periph;
};

// I2C pin definitions
#if CONFIG_MACH_SAM3X
DECL_ENUMERATION_RANGE("i2c_bus", "twi0", 0, 2);
DECL_CONSTANT_STR("BUS_PINS_twi0", "PA18,PA17");
DECL_CONSTANT_STR("BUS_PINS_twi1", "PB13,PB12");
#define PRD_CALC_NUM 4
#elif CONFIG_MACH_SAM4
DECL_ENUMERATION_RANGE("i2c_bus", "twi0", 0, 2);
DECL_CONSTANT_STR("BUS_PINS_twi0", "PA4,PA3");
DECL_CONSTANT_STR("BUS_PINS_twi1", "PB5,PB4");
#define PRD_CALC_NUM 4
#elif CONFIG_MACH_SAME70
DECL_ENUMERATION_RANGE("i2c_bus", "twihs0", 0,3);
DECL_CONSTANT_STR("BUS_PINS_twihs0", "PA4,PA3");
DECL_CONSTANT_STR("BUS_PINS_twihs1", "PB5,PB4");
DECL_CONSTANT_STR("BUS_PINS_twihs2", "PD28,PD27");
#define PRD_CALC_NUM 3
#endif

static const struct twi_info twi_bus[] = {
#if CONFIG_MACH_SAM3X
    { TWI0, ID_TWI0, GPIO('A', 18), GPIO('A', 17), 'A'},
    { TWI1, ID_TWI1, GPIO('B', 13), GPIO('B', 12), 'A'},
#elif CONFIG_MACH_SAM4
    { TWI0, ID_TWI0, GPIO('A', 4), GPIO('A', 3), 'A'},
    { TWI1, ID_TWI1, GPIO('B', 5), GPIO('B', 4), 'A'},
#elif CONFIG_MACH_SAME70
    { TWIHS0, ID_TWIHS0, GPIO('A', 4), GPIO('A', 3), 'A'},
    { TWIHS1, ID_TWIHS1, GPIO('B', 5), GPIO('B', 4), 'A'},
    { TWIHS2, ID_TWIHS2, GPIO('D', 28), GPIO('D', 27), 'C'},
#endif
};

static void
init_pins(uint32_t bus)
{
    const struct twi_info *si = &twi_bus[bus];
    gpio_peripheral(si->scl_pin, si->periph, 1);
    gpio_peripheral(si->sda_pin, si->periph, 1);
    enable_pclock(si->dev_id);
}

static void
i2c_reset(Twi *p_twi)
{
    p_twi->TWI_IDR = 0xFFFFFFFF;
    (void)p_twi->TWI_SR;
    p_twi->TWI_CR = TWI_CR_SWRST;
    (void)p_twi->TWI_RHR;
    p_twi->TWI_CR = TWI_CR_MSDIS;
    p_twi->TWI_CR = TWI_CR_SVDIS;
    p_twi->TWI_CR = TWI_CR_MSEN;
}

static void
i2c_init(uint32_t bus, uint32_t rate)
{
    Twi *p_twi = twi_bus[bus].dev;
    i2c_reset(p_twi);

    uint32_t pclk = get_pclock_frequency(twi_bus[bus].dev_id);
    uint32_t cldiv = 0, chdiv = 0, ckdiv = 0;
    cldiv = pclk / ((rate > 384000 ? 384000 : rate) * 2) - PRD_CALC_NUM;

    while ((cldiv > 255) && (ckdiv < 7)) {
        ckdiv++;
        cldiv /= 2;
    }

    if (rate > 348000) {
        chdiv = pclk / ((2 * rate - 384000) * 2) - PRD_CALC_NUM;
        while ((chdiv > 255) && (ckdiv < 7)) {
            ckdiv++;
            chdiv /= 2;
        }
    } else {
        chdiv = cldiv;
    }
    p_twi->TWI_CWGR = (TWI_CWGR_CLDIV(cldiv) | TWI_CWGR_CHDIV(chdiv)
                       | TWI_CWGR_CKDIV(ckdiv));
}

static uint32_t
addr_to_u32(uint8_t addr_len, uint8_t *addr)
{
    if (addr_len > 3)
        shutdown("Addresses larger than 3 bytes are not supported");
    uint32_t address = 0;
    while (addr_len--)
        address = (address << 8) | *addr++;
    return address;
}

struct i2c_config
i2c_setup(uint32_t bus, uint32_t rate, uint8_t addr)
{
    if (bus >= ARRAY_SIZE(twi_bus) || !rate || rate > 400000)
        shutdown("Invalid i2c_setup parameters!");
    Twi *p_twi = twi_bus[bus].dev;
    init_pins(bus);
    i2c_init(bus, rate);
    return (struct i2c_config){ .twi=p_twi, .cwgr=p_twi->TWI_CWGR,
                               .addr=addr, .bus=bus };
}

// All phases, including STOP, share one deadline. Do not mask interrupts:
// step generation and scheduled outputs must preempt this task-level work.
static int
i2c_wait(Twi *p_twi, uint32_t mask, uint32_t timeout)
{
    for (;;) {
        uint32_t status = p_twi->TWI_SR;
        if (status & TWI_SR_NACK)
            return I2C_BUS_NACK;
        if (!timer_is_before(timer_read_time(), timeout))
            return I2C_BUS_TIMEOUT;
        if (status & mask)
            return I2C_BUS_SUCCESS;
    }
}

static void
i2c_recovery_delay(void)
{
    uint32_t end = timer_read_time() + timer_from_us(5);
    while (timer_is_before(timer_read_time(), end))
        ;
}

// Release a slave left in the middle of a byte. Only drive pins low; a
// released input supplies the high level via the bus pull-ups. Never wait
// indefinitely for a slave holding SCL low, and never retry the data here.
static void
i2c_recover(struct i2c_config config)
{
    Twi *p_twi = config.twi;
    p_twi->TWI_CR = TWI_CR_STOP;
    i2c_reset(p_twi);
    if (CONFIG_ATSAM_I2C_RECOVERY) {
        p_twi->TWI_CR = TWI_CR_MSDIS;
        const struct twi_info *si = &twi_bus[config.bus];
        struct gpio_in scl = gpio_in_setup(si->scl_pin, 1);
        struct gpio_in sda = gpio_in_setup(si->sda_pin, 1);
        struct gpio_out scl_out = { .regs=scl.regs, .bit=scl.bit };
        struct gpio_out sda_out = { .regs=sda.regs, .bit=sda.bit };
        uint32_t end = timer_read_time() + timer_from_us(200);
        for (uint_fast8_t i = 0; i < 9 && !gpio_in_read(sda); i++) {
            gpio_out_reset(scl_out, 0);
            i2c_recovery_delay();
            gpio_in_reset(scl, 1);
            while (!gpio_in_read(scl)) {
                if (!timer_is_before(timer_read_time(), end))
                    goto restore;
            }
            i2c_recovery_delay();
            if (!timer_is_before(timer_read_time(), end))
                goto restore;
        }
        // Generate STOP with SCL high, after first taking it low so that
        // asserting SDA cannot accidentally start another transaction.
        gpio_out_reset(scl_out, 0);
        gpio_out_reset(sda_out, 0);
        i2c_recovery_delay();
        gpio_in_reset(scl, 1);
        while (!gpio_in_read(scl)) {
            if (!timer_is_before(timer_read_time(), end))
                goto restore;
        }
        i2c_recovery_delay();
        gpio_in_reset(sda, 1);
        i2c_recovery_delay();
restore:
        gpio_in_reset(scl, 1);
        gpio_in_reset(sda, 1);
        init_pins(config.bus);
        i2c_reset(p_twi);
    }
    p_twi->TWI_CWGR = config.cwgr;
}

int
i2c_write(struct i2c_config config, uint8_t write_len, uint8_t *write)
{
    if (!write_len)
        return I2C_BUS_SUCCESS;
    Twi *p_twi = config.twi;
    uint32_t timeout = timer_read_time() + timer_from_us(5000);
    p_twi->TWI_CWGR = config.cwgr;
    p_twi->TWI_MMR = TWI_MMR_DADR(config.addr);
    int ret;
    while (write_len--) {
        ret = i2c_wait(p_twi, TWI_SR_TXRDY, timeout);
        if (ret)
            goto fail;
        p_twi->TWI_THR = *write++;
    }
    ret = i2c_wait(p_twi, TWI_SR_TXRDY, timeout);
    if (ret)
        goto fail;
    p_twi->TWI_CR = TWI_CR_STOP;
    ret = i2c_wait(p_twi, TWI_SR_TXCOMP, timeout);
    if (ret)
        goto fail;
    return I2C_BUS_SUCCESS;
fail:
    i2c_recover(config);
    return ret;
}

int
i2c_read(struct i2c_config config, uint8_t reg_len, uint8_t *reg
         , uint8_t read_len, uint8_t *read)
{
    if (!read_len)
        return i2c_write(config, reg_len, reg);
    uint32_t address = addr_to_u32(reg_len, reg);
    Twi *p_twi = config.twi;
    uint32_t timeout = timer_read_time() + timer_from_us(5000);
    uint32_t bytes_to_send = read_len;
    uint8_t stop = 0;
    p_twi->TWI_CWGR = config.cwgr;
    p_twi->TWI_MMR = 0;
    p_twi->TWI_MMR = (TWI_MMR_MREAD | TWI_MMR_DADR(config.addr)
                      | ((reg_len << TWI_MMR_IADRSZ_Pos) & TWI_MMR_IADRSZ_Msk));
    p_twi->TWI_IADR = 0;
    p_twi->TWI_IADR = address;
    if (bytes_to_send == 1) {
        p_twi->TWI_CR = TWI_CR_START | TWI_CR_STOP;
        stop = 1;
    } else {
        p_twi->TWI_CR = TWI_CR_START;
        stop = 0;
    }
    int ret;
    while (bytes_to_send > 0) {
        if (bytes_to_send == 1 && !stop) {
            p_twi->TWI_CR = TWI_CR_STOP;
            stop = 1;
        }
        ret = i2c_wait(p_twi, TWI_SR_RXRDY, timeout);
        if (ret)
            goto fail;
        *read++ = p_twi->TWI_RHR;
        bytes_to_send--;
    }
    ret = i2c_wait(p_twi, TWI_SR_TXCOMP, timeout);
    if (ret)
        goto fail;
    return I2C_BUS_SUCCESS;
fail:
    i2c_recover(config);
    return ret;
}
