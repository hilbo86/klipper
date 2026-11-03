"""Compile the SAM I2C driver against deterministic register/timer models.

Run with python -m unittest test.test_atsam_i2c. HOST_CC may select a host C
compiler. These tests execute the production driver and command code; only
their platform includes are replaced with the small hardware model below.
"""
import os
import pathlib
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


MODEL = r'''
#include <assert.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "i2ccmds.h"

#define CONFIG_MACH_SAM3X 1
#define CONFIG_MACH_SAM4 0
#define CONFIG_MACH_SAME70 0
#define CONFIG_ATSAM_I2C_RECOVERY 1
#define CONFIG_WANT_SOFTWARE_I2C 0
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define DECL_CONSTANT(...)
#define DECL_CONSTANT_STR(...)
#define DECL_ENUMERATION_RANGE(...)
#define DECL_ENUMERATION(...)
#define DECL_COMMAND(...)
#define GPIO(port, pin) (((port) - 'A') * 32 + (pin))

#define TWI_SR_TXCOMP 1u
#define TWI_SR_RXRDY 2u
#define TWI_SR_TXRDY 4u
#define TWI_SR_NACK 0x100u
#define TWI_CR_START 1u
#define TWI_CR_STOP 2u
#define TWI_CR_MSEN 4u
#define TWI_CR_MSDIS 8u
#define TWI_CR_SVDIS 32u
#define TWI_CR_SWRST 128u
#define TWI_MMR_MREAD (1u << 12)
#define TWI_MMR_DADR(a) ((a) << 16)
#define TWI_MMR_IADRSZ_Pos 8
#define TWI_MMR_IADRSZ_Msk (3u << 8)
#define TWI_CWGR_CLDIV(a) (a)
#define TWI_CWGR_CHDIV(a) ((a) << 8)
#define TWI_CWGR_CKDIV(a) ((a) << 16)

typedef struct {
    uint32_t TWI_IDR, TWI_CR, TWI_CWGR, TWI_MMR, TWI_IADR;
    uint32_t mock_sr[1], mock_rhr[1], mock_thr[1];
} Twi;
static Twi mock_twi[2];
#define TWI0 (&mock_twi[0])
#define TWI1 (&mock_twi[1])
#define ID_TWI0 22
#define ID_TWI1 23

// Array-index expressions intercept register reads and THR writes while
// retaining the driver's original C expressions, including their ordering.
static unsigned mock_status(Twi *p_twi);
static unsigned mock_receive(Twi *p_twi);
static unsigned mock_transmit(Twi *p_twi);
#define TWI_SR mock_sr[mock_status(p_twi)]
#define TWI_RHR mock_rhr[mock_receive(p_twi)]
#define TWI_THR mock_thr[mock_transmit(p_twi)]

enum fault { GOOD, STALL_TX, STALL_RX, STALL_STOP, NACK_ADDRESS,
             NACK_DATA, NACK_STOP, NACK_AFTER_READ, SLOW_TRANSFER };
static enum fault fault;
static uint32_t now, elapsed, timer_calls;
static unsigned tx_count, rx_count, reset_count, peripheral_count;
static unsigned scl_low_count, sda_low_count, high_drives;
static int sda_stuck, scl_stuck, release_after_pulses;
static int last_scl_drive_low, first_rx_had_stop;
static struct i2cdev_s mock_i2cdev;
static uint8_t command_write_data[4] = {0x01, 0x23, 0x45, 0x67};
static unsigned response_len, response_status;
static uint8_t response_bytes[32];

static uint32_t timer_from_us(uint32_t us) { return us; }
static uint32_t timer_read_time(void)
{
    // A missing timeout fails promptly rather than hanging the test suite.
    assert(++timer_calls < 20000);
    elapsed++;
    return now++;
}
static int timer_is_before(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) < 0;
}
static void shutdown(const char *message)
{
    fprintf(stderr, "Unexpected shutdown: %s\n", message);
    abort();
}
static void enable_pclock(uint32_t id) { (void)id; }
static uint32_t get_pclock_frequency(uint32_t id)
{
    (void)id;
    return 84000000;
}
static void gpio_peripheral(uint32_t pin, char function, int32_t pullup)
{
    assert(pin == GPIO('A', 18) || pin == GPIO('A', 17)
           || pin == GPIO('B', 13) || pin == GPIO('B', 12));
    assert(function == 'A' && pullup == 1);
    peripheral_count++;
}
struct gpio_in gpio_in_setup(uint8_t pin, int8_t pullup)
{
    assert(pullup == 1);
    return (struct gpio_in){ .regs=(void *)(uintptr_t)pin, .bit=1 };
}
void gpio_in_reset(struct gpio_in pin, int8_t pullup)
{
    assert(pullup == 1);
    if ((uintptr_t)pin.regs == GPIO('B', 13))
        last_scl_drive_low = 0;
}
uint8_t gpio_in_read(struct gpio_in pin)
{
    if ((uintptr_t)pin.regs == GPIO('B', 13))
        return !scl_stuck;
    assert((uintptr_t)pin.regs == GPIO('B', 12));
    if (release_after_pulses && scl_low_count >= release_after_pulses)
        return 1;
    return !sda_stuck;
}
void gpio_out_reset(struct gpio_out pin, uint8_t value)
{
    high_drives += !!value;
    assert(!value); // I2C recovery must never drive either line high.
    if ((uintptr_t)pin.regs == GPIO('B', 13)) {
        scl_low_count++;
        last_scl_drive_low = 1;
    } else {
        assert((uintptr_t)pin.regs == GPIO('B', 12));
        assert(last_scl_drive_low); // STOP setup must not create START.
        sda_low_count++;
    }
}

static unsigned mock_status(Twi *p_twi)
{
    uint32_t status = TWI_SR_TXRDY | TWI_SR_RXRDY;
    if (p_twi->TWI_CR & TWI_CR_STOP)
        status |= TWI_SR_TXCOMP;
    switch (fault) {
    case STALL_TX: status &= ~TWI_SR_TXRDY; break;
    case STALL_RX: status &= ~TWI_SR_RXRDY; break;
    case STALL_STOP: status &= ~TWI_SR_TXCOMP; break;
    case NACK_ADDRESS: status |= TWI_SR_NACK; break;
    case NACK_DATA:
        if (tx_count)
            status |= TWI_SR_NACK;
        break;
    case NACK_STOP:
        if (p_twi->TWI_CR & TWI_CR_STOP)
            status |= TWI_SR_NACK;
        break;
    case NACK_AFTER_READ:
        if (rx_count)
            status |= TWI_SR_NACK;
        break;
    case SLOW_TRANSFER:
        // Each individual byte is below 5ms; their combined duration is not.
        if (elapsed < 5000) {
            elapsed += 1700;
            now += 1700;
        }
        break;
    default: break;
    }
    p_twi->mock_sr[0] = status;
    return 0;
}
static unsigned mock_receive(Twi *p_twi)
{
    if (p_twi->TWI_CR == TWI_CR_SWRST) {
        reset_count++;
        p_twi->TWI_CWGR = 0; // Real SWRST loses the clock divider.
    } else {
        if (!rx_count)
            first_rx_had_stop = !!(p_twi->TWI_CR & TWI_CR_STOP);
        p_twi->mock_rhr[0] = 0xa0 + rx_count++;
    }
    return 0;
}
static unsigned mock_transmit(Twi *p_twi)
{
    (void)p_twi;
    tx_count++;
    return 0;
}
static void *oid_alloc(uint8_t oid, void *type, size_t size)
{
    (void)oid; (void)type;
    assert(size == sizeof(mock_i2cdev));
    return &mock_i2cdev;
}
static void *oid_lookup(uint8_t oid, void *type)
{
    (void)oid; (void)type;
    return &mock_i2cdev;
}
static void *command_decode_ptr(uint32_t encoded)
{
    (void)encoded;
    return command_write_data;
}
static void sendf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    assert(!strcmp(format,
        "i2c_response oid=%c i2c_bus_status=%c response=%*s"));
    (void)va_arg(args, int);
    response_status = va_arg(args, int);
    response_len = va_arg(args, int);
    const uint8_t *data = va_arg(args, const uint8_t *);
    assert(response_len <= sizeof(response_bytes));
    memcpy(response_bytes, data, response_len);
    va_end(args);
}
static int i2c_software_write(struct i2c_software *sw, uint8_t len,
                             uint8_t *data)
{
    (void)sw; (void)len; (void)data;
    abort();
}
static int i2c_software_read(struct i2c_software *sw, uint8_t reg_len,
                            uint8_t *reg, uint8_t len, uint8_t *data)
{
    (void)sw; (void)reg_len; (void)reg; (void)len; (void)data;
    abort();
}
'''


CASES = r'''
static struct i2c_config prepare(void)
{
    memset(mock_twi, 0, sizeof(mock_twi));
    memset(&mock_i2cdev, 0, sizeof(mock_i2cdev));
    fault = GOOD;
    now = elapsed = timer_calls = 0;
    tx_count = rx_count = reset_count = peripheral_count = 0;
    scl_low_count = sda_low_count = high_drives = 0;
    sda_stuck = scl_stuck = release_after_pulses = 0;
    last_scl_drive_low = first_rx_had_stop = 0;
    struct i2c_config config = i2c_setup(1, 100000, 0x48);
    reset_count = peripheral_count = 0;
    return config;
}

static void assert_reusable(struct i2c_config config)
{
    assert(reset_count >= 1);
    assert(((Twi *)config.twi)->TWI_CWGR == config.cwgr);
    assert(high_drives == 0);
    fault = GOOD;
    sda_stuck = scl_stuck = 0;
    uint8_t byte = 0x5a;
    assert(i2c_write(config, 1, &byte) == I2C_BUS_SUCCESS);
    assert(((Twi *)config.twi)->TWI_CWGR == config.cwgr);
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    struct i2c_config config = prepare();
    uint8_t write[] = {1, 2, 3}, read[3] = {0xff, 0xff, 0xff};
    const char *name = argv[1];
    if (!strcmp(name, "success")) {
        assert(i2c_write(config, 3, write) == I2C_BUS_SUCCESS);
        assert(tx_count == 3);
        assert(i2c_read(config, 1, write, 2, read) == I2C_BUS_SUCCESS);
        assert(read[0] == 0xa0 && read[1] == 0xa1);
        assert(rx_count == 2 && !first_rx_had_stop);
        assert(elapsed < 100 && !reset_count && !scl_low_count);
    } else if (!strcmp(name, "tx_stall")
               || !strcmp(name, "rx_stall")
               || !strcmp(name, "stop_stall")
               || !strcmp(name, "read_stop_stall")) {
        fault = !strcmp(name, "tx_stall") ? STALL_TX
              : !strcmp(name, "rx_stall") ? STALL_RX : STALL_STOP;
        int is_read = !strcmp(name, "rx_stall")
                      || !strcmp(name, "read_stop_stall");
        int ret = is_read ? i2c_read(config, 1, write, 2, read)
                          : i2c_write(config, 3, write);
        assert(ret == I2C_BUS_TIMEOUT);
        assert(elapsed >= 5000 && elapsed < 5250);
        assert_reusable(config);
    } else if (!strcmp(name, "address_nack")
               || !strcmp(name, "data_nack")
               || !strcmp(name, "stop_nack")
               || !strcmp(name, "read_nack")) {
        fault = !strcmp(name, "data_nack") ? NACK_DATA
              : !strcmp(name, "stop_nack") ? NACK_STOP : NACK_ADDRESS;
        int ret = !strcmp(name, "read_nack")
                  ? i2c_read(config, 1, write, 2, read)
                  : i2c_write(config, 3, write);
        assert(ret == I2C_BUS_NACK);
        assert(elapsed < 250);
        if (fault == NACK_DATA)
            assert(tx_count == 1); // No retry or duplicate side effect.
        if (!strcmp(name, "read_nack"))
            assert(read[0] == 0xff && !rx_count);
        assert_reusable(config);
    } else if (!strcmp(name, "wraparound")) {
        now = UINT32_MAX - 100;
        fault = STALL_TX;
        assert(i2c_write(config, 1, write) == I2C_BUS_TIMEOUT);
        assert(elapsed >= 5000 && elapsed < 5250);
        assert_reusable(config);
    } else if (!strcmp(name, "shared_deadline")) {
        fault = SLOW_TRANSFER;
        assert(i2c_write(config, 3, write) == I2C_BUS_TIMEOUT);
        assert(tx_count == 2);
        assert(elapsed < 5400);
        assert_reusable(config);
    } else if (!strcmp(name, "zero_register")) {
        assert(i2c_read(config, 0, NULL, 1, read) == I2C_BUS_SUCCESS);
        assert(((Twi *)config.twi)->TWI_IADR == 0);
        assert(read[0] == 0xa0 && rx_count == 1 && first_rx_had_stop);
    } else if (!strcmp(name, "zero_write")) {
        assert(i2c_write(config, 0, NULL) == I2C_BUS_SUCCESS);
        assert(!tx_count && !rx_count && !elapsed && !reset_count);
    } else if (!strcmp(name, "client_clock")) {
        struct i2c_config fast = i2c_setup(1, 400000, 0x49);
        assert(config.cwgr != fast.cwgr);
        assert(i2c_write(config, 1, write) == I2C_BUS_SUCCESS);
        assert(((Twi *)config.twi)->TWI_CWGR == config.cwgr);
        assert(i2c_write(fast, 1, write) == I2C_BUS_SUCCESS);
        assert(((Twi *)fast.twi)->TWI_CWGR == fast.cwgr);
    } else if (!strcmp(name, "sda_stuck")
               || !strcmp(name, "sda_release")) {
        fault = NACK_ADDRESS;
        sda_stuck = 1;
        if (!strcmp(name, "sda_release"))
            release_after_pulses = 3;
        assert(i2c_write(config, 1, write) == I2C_BUS_NACK);
        assert(scl_low_count == (release_after_pulses ? 4 : 10));
        assert(sda_low_count == 1 && !high_drives);
        assert(elapsed < 250 && peripheral_count == 2);
        assert_reusable(config);
    } else if (!strcmp(name, "scl_stuck")) {
        fault = NACK_ADDRESS;
        sda_stuck = scl_stuck = 1;
        assert(i2c_write(config, 1, write) == I2C_BUS_NACK);
        assert(scl_low_count == 1 && !high_drives);
        assert(elapsed >= 200 && elapsed < 250);
        assert(peripheral_count == 2);
        assert_reusable(config);
    } else if (!strcmp(name, "failed_response")
               || !strcmp(name, "partial_response")
               || !strcmp(name, "good_response")) {
        mock_i2cdev.i2c_hw = config;
        fault = !strcmp(name, "failed_response") ? NACK_ADDRESS
                : !strcmp(name, "partial_response") ? NACK_AFTER_READ : GOOD;
        uint32_t args[] = {5, 1, 0, 2};
        command_i2c_transfer(args);
        if (fault == GOOD) {
            assert(response_status == I2C_BUS_SUCCESS && response_len == 2);
            assert(response_bytes[0] == 0xa0 && response_bytes[1] == 0xa1);
        } else {
            assert(response_status == I2C_BUS_NACK && response_len == 0);
            if (fault == NACK_AFTER_READ)
                assert(rx_count == 1); // Discard even a partially filled read.
        }
    } else {
        fprintf(stderr, "Unknown scenario: %s\n", name);
        return 2;
    }
    puts(name);
    return 0;
}
'''


class TestATSAMI2C(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        candidate = os.environ.get('HOST_CC')
        if candidate is None:
            candidate = next((shutil.which(c) for c in ('cc', 'gcc', 'clang')
                              if shutil.which(c)), None)
        local_cc = ROOT / 'out/test-toolchain/mingw64/bin/gcc.exe'
        if candidate is None and local_cc.is_file():
            candidate = str(local_cc)
        if candidate is None:
            raise unittest.SkipTest('Host C compiler required (set HOST_CC)')
        cls.temp = tempfile.TemporaryDirectory(prefix='klipper-i2c-')
        cls.addClassCleanup(cls.temp.cleanup)
        directory = pathlib.Path(cls.temp.name)
        (directory / 'board').mkdir()
        gpio_header = (ROOT / 'src/atsam/gpio.h').as_posix()
        (directory / 'board/gpio.h').write_text(
            '#include "%s"\n' % gpio_header, encoding='utf-8')
        sources = []
        for filename in ('src/atsam/i2c.c', 'src/i2ccmds.c'):
            source = (ROOT / filename).read_text(encoding='utf-8')
            sources.append(re.sub(r'^#include[^\n]*\n', '', source,
                                  flags=re.MULTILINE))
        source_file = directory / 'i2c_faults.c'
        source_file.write_text(MODEL + '\n'.join(sources) + CASES,
                               encoding='utf-8')
        cls.executable = directory / ('i2c_faults.exe' if os.name == 'nt'
                                      else 'i2c_faults')
        environment = dict(os.environ)
        environment['PATH'] = str(pathlib.Path(candidate).resolve().parent) \
            + os.pathsep + environment.get('PATH', '')
        command = [candidate, '-std=gnu11', '-Wall', '-Werror',
                   '-Wno-unused-function', '-Wno-unused-variable',
                   '-I', str(directory), '-I', str(ROOT / 'src'),
                   str(source_file), '-o', str(cls.executable)]
        result = subprocess.run(command, text=True, capture_output=True,
                                env=environment, timeout=30)
        if result.returncode:
            raise RuntimeError('C harness failed to compile:\n' + result.stdout
                               + result.stderr)

    def check_scenario(self, name):
        result = subprocess.run([str(self.executable), name], text=True,
                                capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_success_and_clock_configuration(self):
        for name in ('success', 'zero_register', 'zero_write', 'client_clock'):
            with self.subTest(name=name):
                self.check_scenario(name)

    def test_all_waits_have_one_wrap_safe_deadline(self):
        for name in ('tx_stall', 'rx_stall', 'stop_stall', 'read_stop_stall',
                     'wraparound', 'shared_deadline'):
            with self.subTest(name=name):
                self.check_scenario(name)

    def test_nacks_return_without_retry_and_next_transfer_succeeds(self):
        for name in ('address_nack', 'data_nack', 'stop_nack', 'read_nack'):
            with self.subTest(name=name):
                self.check_scenario(name)

    def test_bus_clear_is_bounded_and_never_drives_high(self):
        for name in ('sda_stuck', 'sda_release', 'scl_stuck'):
            with self.subTest(name=name):
                self.check_scenario(name)

    def test_failed_response_does_not_expose_partial_or_stack_bytes(self):
        for name in ('failed_response', 'partial_response', 'good_response'):
            with self.subTest(name=name):
                self.check_scenario(name)


if __name__ == '__main__':
    unittest.main()
