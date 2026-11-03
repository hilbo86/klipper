"""Execute the production decoder, scheduler client and AVR IRQ backend.

The register model does not simulate voltage / rise time or interrupt latency.
It deterministically checks digital edges, wraparound, lifecycle and conflicts.
Run: python -m unittest test.test_dht
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
#include <setjmp.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dht.h"

// Fail without invoking Windows' interactive CRT assertion / crash dialogs.
#undef assert
#define assert(expr) do { if (!(expr)) { \
    fprintf(stderr,"line %d: %s\n",__LINE__,#expr); exit(3); \
} } while (0)

#define CONFIG_MACH_atmega328p 1
#define CONFIG_CLOCK_FREQ 16000000
#define CONFIG_HAVE_STEPPER 0
#define CONFIG_WANT_HARD_PWM 0
#define DECL_CONSTANT(...)
#define DECL_COMMAND(...)
#define DECL_TASK(...)
#define DECL_SHUTDOWN(...)
#define GPIO(p, n) (((p)-'A')*8+(n))
#define container_of(p, t, m) ((t*)((char*)(p)-offsetof(t, m)))
#define ISR(name) void name(void)
#define barrier() __asm__ __volatile__("" ::: "memory")
enum { PCINT5=5, PCIE0=0, PCIF0=0, INT0=0, INTF0=0,
       ISC00=0, ISC01=1, SPE=6 };
static uint8_t PCICR, PCIFR, EICRA, EIFR, SPCR;
static uint8_t model_pcmsk0, model_eimsk;
static uint8_t DDRB, DDRD, PORTB, PORTD, PINB, PIND;
static struct dht_gpio *receiver;
static unsigned inject_arm_rise;
static uint8_t *irq_mask_register(uint8_t *reg) {
    if (inject_arm_rise && receiver) {
        PINB=PIND=0xff; inject_arm_rise=0;
    }
    return reg;
}
#define PCMSK0 (*irq_mask_register(&model_pcmsk0))
#define EIMSK (*irq_mask_register(&model_eimsk))
static uint16_t model_tcnt1;
static unsigned inject_rise, inject_pin;
static void rise_if_released(void) {
    unsigned output=inject_pin==GPIO('B',5)
        ? DDRB&(1<<5) : DDRD&(1<<2);
    if (inject_rise==2 && !output) {
        PINB=PIND=0xff; inject_rise=0;
    }
}
static uint16_t *timer_register(void) {
    rise_if_released(); return &model_tcnt1;
}
#define TCNT1 (*timer_register())
static void *model_memset(void *data, int value, size_t size) {
    void *result=memset(data,value,size);
    // A pull-up edge while decode_start clears the receive buffer.
    if (inject_rise==1) { inject_rise=2; rise_if_released(); }
    return result;
}
#define memset model_memset
struct timer { struct timer *next; uint_fast8_t (*func)(struct timer*);
               uint32_t waketime; };
struct task_wake { uint8_t wake; };
enum { SF_DONE, SF_RESCHEDULE };
static struct timer *scheduled;
static uint32_t now;
static unsigned locked, reports, report_status, report_oid;
static int report_temp, report_humidity;
static uint32_t report_clock;
static unsigned diagnostic_reports, diagnostic_present, diagnostic_state;
static unsigned diagnostic_bits, diagnostic_ticks, diagnostic_level;
static uint32_t diagnostic_clock;
typedef unsigned irqstatus_t;
static irqstatus_t irq_save(void) { unsigned old=locked; locked=1; return old; }
static void irq_restore(irqstatus_t old) { locked=old; }
static uint32_t timer_read_time(void) { return now; }
static uint32_t timer_from_us(uint32_t us) { return us*16; }
static int timer_is_before(uint32_t a, uint32_t b) { return (int32_t)(a-b)<0; }
static void sched_wake_task(struct task_wake *w) { w->wake=1; }
static int sched_check_wake(struct task_wake *w) {
    int v=w->wake; w->wake=0; return v;
}
static void sched_add_timer(struct timer *t) {
    assert(locked && !scheduled && timer_is_before(now, t->waketime));
    scheduled=t;
}
static void sched_del_timer(struct timer *t) {
    if (scheduled==t) scheduled=NULL;
}
static union { long double align; uint8_t data[256]; } arena;
static unsigned object_count, object_oid, allocated;
static void *oid_alloc(unsigned oid, void (*func)(uint32_t*), unsigned size) {
    assert(!object_count && size<=sizeof(arena));
    object_count=1; object_oid=oid; allocated=size;
    memset(&arena, 0, sizeof(arena)); return &arena;
}
static void *oid_lookup(unsigned oid, void (*func)(uint32_t*)) {
    assert(object_count && oid==object_oid); return &arena;
}
#define foreach_oid(oid, ptr, func) \
    for (oid=object_oid; oid<object_oid+object_count \
         && (((ptr)=(void*)&arena),1); oid++)
static jmp_buf failure;
static int expect_shutdown;
static const char *shutdown_reason;
static void model_shutdown(const char *reason) {
    shutdown_reason=reason;
    if (!expect_shutdown) { fprintf(stderr, "%s\n", reason); exit(3); }
    longjmp(failure, 1);
}
#define shutdown(msg) model_shutdown(msg)
static void model_send(const char *format, ...) {
    assert(!locked);
    va_list args; va_start(args, format);
    if (!strncmp(format,"dht_diagnostics ",16)) {
        assert(va_arg(args, int)==object_oid);
        diagnostic_clock=va_arg(args, uint32_t);
        diagnostic_present=va_arg(args, int);
        diagnostic_state=va_arg(args, int);
        diagnostic_bits=va_arg(args, int);
        diagnostic_ticks=va_arg(args, int);
        diagnostic_level=va_arg(args, int);
        diagnostic_reports++; va_end(args); return;
    }
    report_oid=va_arg(args, int); report_clock=va_arg(args, uint32_t);
    report_status=va_arg(args, int); report_temp=va_arg(args, int);
    report_humidity=va_arg(args, int); va_end(args); reports++;
}
#define sendf(...) model_send(__VA_ARGS__)
'''

CASES = r'''
static struct dht *sensor(void) { return (void*)&arena; }
static void safe_gpio(void) {
    assert(!(DDRB & PORTB & (1<<5)) && !(DDRD & PORTD & (1<<2)));
}
static void released(void) {
    safe_gpio(); assert(!receiver);
    if (sensor()->gpio.pin==GPIO('B',5))
        assert(!(DDRB&(1<<5)) && !(PCMSK0&(1<<PCINT5)));
    else assert(!(DDRD&(1<<2)) && !(EIMSK&(1<<INT0)));
}
static void advance_ticks(uint32_t delta) {
    uint32_t end=now+delta; unsigned iterations=0;
    while (scheduled && !timer_is_before(end, scheduled->waketime)) {
        assert(++iterations<1000);
        now=scheduled->waketime; TCNT1=now;
        struct timer *t=scheduled;
        irqstatus_t flag=irq_save();
        int result=t->func(t); irq_restore(flag);
        if (result==SF_DONE) scheduled=NULL;
        else assert(timer_is_before(now, t->waketime));
        safe_gpio();
    }
    now=end; TCNT1=now;
}
static void advance(unsigned us) { advance_ticks(us*16); }
static void configure(unsigned type, unsigned pin) {
    uint32_t args[]={7,pin,type,0}; command_config_dht(args);
}
static void query(unsigned rest_us) {
    uint32_t args[]={7,now+16000,rest_us*16}; command_query_dht(args);
}
static void begin(unsigned type, unsigned pin, int release_high) {
    configure(type,pin); query(5000000);
    assert(scheduled->waketime==now+32000000);
    advance_ticks(scheduled->waketime-now);
    assert(sensor()->phase==DHT_RELEASE && !receiver);
    assert((pin==GPIO('B',5) ? DDRB&(1<<5) : DDRD&(1<<2)));
    if (!release_high) PINB=PIND=0;
    advance_ticks(scheduled->waketime-now);
    assert(sensor()->phase==DHT_RECEIVE && receiver);
    safe_gpio();
}
static void edge(unsigned us, unsigned level) {
    advance(us);
    if (sensor()->gpio.pin==GPIO('B',5)) {
        PINB=level ? 0xff : 0;
        if (PCMSK0&(1<<PCINT5)) {
            irqstatus_t flag=irq_save(); PCINT0_vect(); irq_restore(flag);
        }
    } else {
        PIND=level ? 0xff : 0;
        if (EIMSK&(1<<INT0)) {
            irqstatus_t flag=irq_save(); INT0_vect(); irq_restore(flag);
        }
    }
    safe_gpio();
}
static void preamble(void) { edge(30,0); edge(80,1); edge(80,0); }
static void bit(unsigned value, unsigned zero, unsigned one) {
    edge(50,1); edge(value ? one : zero,0);
}
static void frame(const uint8_t *b, unsigned zero, unsigned one) {
    for (unsigned n=0; n<40; n++) bit((b[n/8]>>(7-n%8))&1,zero,one);
}
static void checksum(uint8_t *b) { b[4]=b[0]+b[1]+b[2]+b[3]; }
static void reset_model(void) {
    inject_rise=inject_arm_rise=0;
    scheduled=NULL; now=TCNT1=0; locked=reports=0;
    diagnostic_reports=0;
    object_count=object_oid=allocated=0; active_dht=NULL; receiver=NULL;
    dht_wake.wake=0;
    PINB=PIND=0xff; PCMSK0=PCICR=EIMSK=EICRA=SPCR=0;
    DDRB=DDRD=PORTB=PORTD=0;
}
int main(int argc, char **argv) {
    assert(argc==2); const char *name=argv[1]; reset_model();
    if (!strcmp(name,"formats")) {
        uint8_t bytes[][5]={{55,3,24,6,0},{2,146,1,13,0},
                           {2,146,128,101,0}};
        int expected[]={246,269,-101};
        for (unsigned i=0;i<3;i++) {
            reset_model(); uint8_t *b=bytes[i]; checksum(b);
            begin(i ? DHT22 : DHT11, i==2 ? GPIO('D',2) : GPIO('B',5),1);
            uint32_t start=sensor()->start_clock; preamble(); frame(b,27,70);
            assert(sensor()->decoder.bits==40 && sensor()->pending);
            released(); assert(reports==0); dht_task();
            assert(reports==1 && report_status==DHT_OK && report_oid==7);
            assert(report_temp==expected[i]
                   && report_humidity==(i ? 658 : 553));
            assert(report_clock==start); dht_task(); assert(reports==1);
            edge(50,1); assert(!sensor()->pending); advance(500); dht_task();
            assert(reports==1 && sensor()->phase==DHT_START);
            assert(scheduled->waketime==start+80000000);
        }
    } else if (!strcmp(name,"patterns")) {
        for (unsigned pattern=0;pattern<4;pattern++) {
            struct dht_decoder d; uint8_t data[5];
            unsigned value=pattern==0 ? 0 : pattern==1 ? 255
                : pattern==2 ? 0xaa : 0x55;
            memset(data,value,5);
            uint16_t clock=65000; dht_decode_start(&d,1,clock);
            assert(dht_decode_edge(&d,0,clock+=30*16)==DHT_PENDING);
            assert(dht_decode_edge(&d,1,clock+=80*16)==DHT_PENDING);
            assert(dht_decode_edge(&d,0,clock+=80*16)==DHT_PENDING);
            for (unsigned n=0;n<40;n++) {
                assert(dht_decode_edge(&d,1,clock+=50*16)==DHT_PENDING);
                unsigned value=(data[n/8]>>(7-n%8))&1;
                unsigned status=dht_decode_edge(&d,0,clock+=(value?70:27)*16);
                assert(status==(n==39?DHT_OK:DHT_PENDING));
            }
            assert(!memcmp(data,d.data,5));
            assert(dht_decode_edge(&d,1,clock+50*16)==DHT_PULSE_ERROR);
        }
    } else if (!strcmp(name,"threshold_jitter")) {
        uint8_t b[]={2,146,128,101,0}; checksum(b);
        for (unsigned i=0;i<20;i++) {
            reset_model(); begin(DHT22,GPIO('D',2),1);
            edge(20+i,0); edge(65+i,1); edge(65+i,0);
            frame(b,30+i,50+i); dht_task();
            assert(reports==1 && report_status==DHT_OK && report_temp==-101);
        }
    } else if (!strcmp(name,"bad_frames")) {
        for (unsigned mode=0;mode<7;mode++) {
            reset_model(); begin(DHT11,GPIO('B',5),1);
            uint8_t b[]={55,0,24,0,0};
            if (mode==1) b[2]=51;
            if (mode==2) b[1]=10;
            if (mode==3) b[0]=19;
            if (mode==4) b[2]=0x81;
            if (mode==5) b[3]=0x80;
            checksum(b); if (!mode) b[4]^=1;
            preamble(); frame(b,27,70); dht_task();
            assert(report_status==(mode==0?DHT_CHECKSUM:
                                  mode==6?DHT_OK:DHT_OUT_OF_RANGE));
            if (report_status) assert(report_temp==0 && report_humidity==0);
        }
        int16_t t; uint16_t h;
        uint8_t b[]={3,233,0,0,236}; // 100.1% RH, correct CRC
        assert(dht_decode_frame(DHT22,b,&t,&h)==DHT_OUT_OF_RANGE);
    } else if (!strcmp(name,"timeouts")) {
        for (unsigned stage=0;stage<5;stage++) {
            reset_model(); begin(DHT22,GPIO('D',2),stage!=1);
            if (stage>=2) edge(30,0);
            if (stage>=3) edge(80,1);
            if (stage==4) { edge(80,0); bit(1,27,70); }
            advance(500); dht_task(); released();
            assert(reports==1 && report_status==
                   (stage<2?DHT_NO_RESPONSE:DHT_TIMEOUT));
            advance(8000); dht_task(); assert(reports==1);
        }
    } else if (!strcmp(name,"long_gap_alias")) {
        begin(DHT22,GPIO('D',2),1); edge(30,0);
        edge(4096+80,1); // Would look like 80us with only 16-bit subtraction.
        dht_task(); assert(reports==1 && report_status==DHT_TIMEOUT);
        released();
    } else if (!strcmp(name,"bad_edges")) {
        for (unsigned mode=0;mode<5;mode++) {
            reset_model(); begin(DHT22,GPIO('D',2),1);
            if (mode==0) { edge(2,0); }
            if (mode==1) { edge(30,0); edge(20,1); }
            if (mode==2) { preamble(); edge(5,1); }
            if (mode==3) { preamble(); edge(50,1); edge(5,0); }
            if (mode==4) { preamble(); edge(50,1); edge(120,0); }
            dht_task(); assert(reports==1 && report_status==DHT_PULSE_ERROR);
            released();
        }
    } else if (!strcmp(name,"lost_edges")) {
        begin(DHT22,GPIO('D',2),1); preamble(); edge(50,1);
        advance(27); PIND=0; // Drop one falling IRQ; next rise merges.
        edge(50,1); advance(500); dht_task();
        assert(reports==1 && report_status==DHT_TIMEOUT); released();
    } else if (!strcmp(name,"release_edge")) {
        begin(DHT22,GPIO('D',2),0); edge(4,1);
        uint8_t b[]={2,146,1,13,162};
        preamble(); frame(b,27,70); dht_task(); assert(report_status==DHT_OK);
    } else if (!strcmp(name,"release_rise_during_init")
               || !strcmp(name,"release_rise_at_irq_enable")) {
        for (unsigned pin=0;pin<2;pin++) {
            reset_model();
            inject_pin=pin ? GPIO('D',2) : GPIO('B',5);
            configure(DHT11,inject_pin); query(5000000);
            advance_ticks(scheduled->waketime-now);
            PINB=PIND=0;
            if (!strcmp(name,"release_rise_during_init")) inject_rise=1;
            else inject_arm_rise=1;
            advance_ticks(scheduled->waketime-now);
            uint8_t b[]={23,0,22,0,45};
            preamble(); frame(b,27,70); dht_task();
            assert(reports==1 && report_status==DHT_OK);
        }
    } else if (!strcmp(name,"bus_busy")) {
        configure(DHT11,GPIO('B',5)); PINB=0; query(5000000);
        advance_ticks(scheduled->waketime-now); dht_task(); released();
        assert(reports==1 && report_status==DHT_BUS_BUSY);
    } else if (!strcmp(name,"pulse_diagnostics")) {
        begin(DHT11,GPIO('B',5),1);
        uint32_t args[]={7}; command_query_dht_diagnostics(args);
        assert(diagnostic_reports==1 && !diagnostic_present);
        uint32_t start=sensor()->start_clock;
        preamble(); bit(0,27,70); edge(50,1); edge(5,0);
        dht_task(); assert(report_status==DHT_PULSE_ERROR);
        assert(diagnostic_reports==1); // No unsolicited debug messages.
        command_query_dht_diagnostics(args);
        assert(diagnostic_reports==2 && diagnostic_present);
        assert(diagnostic_state==DHT_BIT_HIGH && diagnostic_bits==1);
        assert(diagnostic_ticks==5*16 && diagnostic_level==0);
        assert(diagnostic_clock==start && reports==1);
        query(0); command_query_dht_diagnostics(args);
        assert(diagnostic_present && diagnostic_clock==start);
    } else if (!strcmp(name,"lifecycle")) {
        for (unsigned action=0;action<3;action++)
        for (unsigned stage=0;stage<9;stage++) {
            reset_model(); configure(DHT22,GPIO('D',2)); query(5000000);
            if (stage>=1) advance_ticks(scheduled->waketime-now);
            if (stage>=2) advance_ticks(scheduled->waketime-now);
            if (stage>=3) edge(30,0);
            if (stage>=4) edge(80,1);
            if (stage>=5) edge(80,0);
            if (stage>=6) edge(50,1);
            if (stage>=7) edge(27,0);
            if (stage>=8) {
                uint8_t b[]={2,146,1,13,162};
                for (unsigned n=1;n<40;n++) bit((b[n/8]>>(7-n%8))&1,27,70);
            }
            if (action==0) { query(0); query(0); assert(!scheduled); }
            if (action==1) {
                // Klipper removes user timers BEFORE calling shutdown hooks.
                scheduled=NULL; dht_shutdown(); dht_shutdown();
                assert(!scheduled);
            }
            if (action==2) {
                query(5000000); query(5000000); assert(scheduled);
            }
            released(); dht_task(); assert(reports==0);
            PINB=PIND=255; query(5000000); assert(scheduled);
        }
    } else if (!strcmp(name,"wrap32")) {
        now=0xff000000; TCNT1=now;
        begin(DHT22,GPIO('D',2),1); advance(500); dht_task();
        assert(reports==1 && report_status==DHT_NO_RESPONSE);
        assert(scheduled && timer_is_before(now, scheduled->waketime));
    } else if (!strcmp(name,"pending_race")) {
        begin(DHT22,GPIO('D',2),1); uint8_t b[]={2,146,1,13,162};
        preamble(); frame(b,27,70); advance(500);
        assert(sensor()->phase==DHT_START);
        uint32_t start=sensor()->start_clock;
        advance_ticks(scheduled->waketime-now); // Deliberately starve task.
        assert(sensor()->start_clock==start && sensor()->pending);
        dht_task(); dht_task(); assert(reports==1 && report_status==DHT_OK);
    } else if (!strcmp(name,"preserve_registers")) {
        configure(DHT11,GPIO('B',5));
        DDRB=0x01; PORTB=0x81; DDRD=0xaa; PORTD=0x55;
        PCMSK0=1; PCICR=0x87; EIMSK=2; EICRA=0xfc;
        irqstatus_t f=irq_save(); dht_gpio_low(&sensor()->gpio);
        dht_gpio_stop(&sensor()->gpio); irq_restore(f);
        assert(DDRB==1 && PORTB==0x81 && DDRD==0xaa && PORTD==0x55);
        assert(PCMSK0==1 && PCICR==0x87 && EIMSK==2 && EICRA==0xfc);
        reset_model(); configure(DHT22,GPIO('D',2)); EICRA=0xfc; EIMSK=2;
        struct dht_decoder d; dht_gpio_receive(&sensor()->gpio,&d);
        assert(EICRA==0xfd && EIMSK==3 && EIFR==(1<<INTF0));
        dht_gpio_stop(&sensor()->gpio); assert(EIMSK==2);
    } else if (!strcmp(name,"pcint_filter")) {
        begin(DHT11,GPIO('B',5),1); unsigned state=sensor()->decoder.state;
        PINB^=1; PCINT0_vect(); assert(sensor()->decoder.state==state);
        assert(!sensor()->pending);
    } else if (!strcmp(name,"restart_long_idle")) {
        configure(DHT22,GPIO('D',2)); query(0);
        advance_ticks(0x80000000u); query(5000000);
        assert(scheduled->waketime==now+16000);
        advance_ticks(scheduled->waketime-now);
        uint32_t start=sensor()->start_clock;
        query(0); query(5000000);
        assert(scheduled->waketime==start+32000016);
    } else if (!strcmp(name,"pullup")) {
        uint32_t args[]={7,GPIO('B',5),DHT11,1};
        command_config_dht(args); released(); assert(PORTB&(1<<5));
        irqstatus_t f=irq_save();
        dht_gpio_low(&sensor()->gpio); safe_gpio();
        assert(!(PORTB&(1<<5)) && (DDRB&(1<<5)));
        dht_gpio_stop(&sensor()->gpio); irq_restore(f); released();
        assert(PORTB&(1<<5));
    } else if (!strcmp(name,"config_errors")) {
        for (unsigned mode=0;mode<9;mode++) {
            reset_model(); expect_shutdown=1;
            if (!setjmp(failure)) {
                if (mode==0) configure(DHT22,GPIO('B',4));
                if (mode==1) { EIMSK=1; configure(DHT22,GPIO('D',2)); }
                if (mode==2) { PCMSK0=2; configure(DHT11,GPIO('B',5)); }
                if (mode==3) { SPCR=1<<SPE; configure(DHT11,GPIO('B',5)); }
                if (mode==4) configure(33,GPIO('D',2));
                if (mode==5) {
                    configure(DHT22,GPIO('D',2)); configure(DHT11,GPIO('B',5));
                }
                if (mode==6) { configure(DHT22,GPIO('D',2)); query(2000000); }
                if (mode==7) { configure(DHT11,GPIO('D',2)); query(1000000); }
                if (mode==8) configure(DHT11,GPIO('B',5)+256);
                assert(!"configuration accepted unexpectedly");
            }
            assert(shutdown_reason && strstr(shutdown_reason,"DHT:"));
            expect_shutdown=0;
        }
    } else return 2;
    puts(name); return 0;
}
'''


class TestDHTFirmware(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = os.environ.get('HOST_CC') or shutil.which('gcc')
        local = ROOT / 'out/test-toolchain/mingw64/bin/gcc.exe'
        if compiler is None and local.is_file():
            compiler = str(local)
        if compiler is None:
            raise unittest.SkipTest('Set HOST_CC to a host C compiler')
        cls.temp = tempfile.TemporaryDirectory(prefix='klipper-dht-')
        cls.addClassCleanup(cls.temp.cleanup)
        directory = pathlib.Path(cls.temp.name)
        sources = []
        for name in ('src/dht_protocol.c', 'src/dht.c', 'src/avr/dht_avr.c'):
            source = (ROOT / name).read_text(encoding='utf-8')
            sources.append(re.sub(r'^#include[^\n]*\n', '', source,
                                  flags=re.MULTILINE))
        source = directory / 'dht_model.c'
        source.write_text(MODEL + '\n'.join(sources) + CASES, encoding='utf-8')
        cls.executable = directory / ('dht.exe' if os.name == 'nt' else 'dht')
        cls.environment = dict(os.environ)
        cls.environment['PATH'] = str(pathlib.Path(compiler).resolve().parent) \
            + os.pathsep + cls.environment.get('PATH', '')
        result = subprocess.run(
            [compiler, '-std=gnu11', '-O2', '-Wall', '-Werror',
             '-Wno-unused-function', '-Wno-unused-variable',
             '-I', str(ROOT / 'src'), str(source), '-o', str(cls.executable)],
            capture_output=True, text=True, env=cls.environment, timeout=30)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)

    def test_production_code(self):
        for scenario in ('formats', 'patterns', 'threshold_jitter',
                         'bad_frames', 'timeouts', 'long_gap_alias',
                         'bad_edges', 'lost_edges',
                         'release_edge', 'release_rise_during_init',
                         'release_rise_at_irq_enable',
                         'bus_busy', 'pulse_diagnostics', 'lifecycle', 'wrap32',
                         'pending_race', 'preserve_registers', 'pcint_filter',
                         'restart_long_idle', 'pullup', 'config_errors'):
            with self.subTest(scenario=scenario):
                result = subprocess.run([str(self.executable), scenario],
                                        capture_output=True, text=True,
                                        env=self.environment, timeout=5)
                self.assertEqual(result.returncode, 0,
                                 result.stdout + result.stderr)


if __name__ == '__main__':
    unittest.main()
