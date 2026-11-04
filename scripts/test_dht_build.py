#!/usr/bin/env python3
"""Build DHT on/off and inspect the real AVR ELF, without touching hardware.

Run from the repository root: python3 scripts/test_dht_build.py
Requires make, AVR GCC/binutils, and a POSIX shell. On Windows the existing
PlatformIO AVR toolchain, local test-toolchain make and Git shell are detected.
Files are generated under out/dht-build; existing profiles stay intact.
"""
import json
import functools
import os
import pathlib
import re
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'out/dht-build'


def run(command, environment, log=None):
    command = list(command)
    executable = shutil.which(command[0], path=environment['PATH'])
    command[0] = executable or command[0]
    result = subprocess.run(command, cwd=ROOT, env=environment, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if log is not None:
        log.write_text(result.stdout, encoding='utf-8')
    if result.returncode:
        raise RuntimeError('Command failed: %r\n%s' % (command, result.stdout))
    return result.stdout


def irq_cycle_bound(disassembly):
    """Conservative longest CFG path, including called functions and RETI.

    Branch conditions are unconstrained, so mutually exclusive paths may be
    included. Reject loops / unknown opcodes instead of understating the cost.
    Other interrupts and entry latency due to interrupted code are excluded.
    """
    instructions, symbols = {}, {}
    for line in disassembly.splitlines():
        label = re.match(r'^([0-9a-f]+) <([^>]+)>:', line)
        if label:
            symbols[label[2]] = int(label[1], 16)
        instruction = re.match(
            r'^\s*([0-9a-f]+):\s+((?:[0-9a-f]{2}\s+)+)'
            r'\s*(\w+)\s*(.*)', line)
        if instruction:
            addr, binary, op, args = instruction.groups()
            instructions[int(addr, 16)] = (len(binary.split()), op, args)
    visiting = set()

    @functools.lru_cache(None)
    def cost(addr):
        if addr in visiting:
            raise ValueError('Loop in IRQ path at 0x%x' % addr)
        visiting.add(addr)
        length, op, args = instructions[addr]
        next_addr = addr + length
        target = re.search(r'0x([0-9a-f]+)', args.split(';', 1)[-1])
        destination = int(target[1], 16) if target else None
        if op in ('ret', 'reti'):
            cycles = 4
        elif op in ('jmp', 'rjmp'):
            cycles = (3 if op == 'jmp' else 2) + cost(destination)
        elif op in ('call', 'rcall'):
            cycles = (4 if op == 'call' else 3) + cost(destination)
            cycles += cost(next_addr)
        elif op.startswith('br'):
            cycles = max(1 + cost(next_addr), 2 + cost(destination))
        elif op in ('cpse', 'sbrc', 'sbrs', 'sbic', 'sbis'):
            skip_length = instructions[next_addr][0]
            cycles = max(1 + cost(next_addr),
                         1 + skip_length // 2 + cost(next_addr + skip_length))
        else:
            one = ('mov movw ldi in out eor and or andi ori cpi cp cpc '
                   'sub subi sbc sbci add adc lsr lsl ror rol bst bld '
                   'inc dec com neg clr ser nop cli sei').split()
            two = ('push pop ld ldd lds st std sts adiw sbiw sbi cbi '
                   'mul muls').split()
            if op not in one + two:
                raise ValueError('Unknown instruction in IRQ: ' + op)
            cycles = (1 if op in one else 2) + cost(next_addr)
        visiting.remove(addr)
        return cycles

    # Four cycles hardware entry and three for the vector-table JMP.
    return {name: cost(symbols[name]) + 7
            for name in ('__vector_1', '__vector_3')}


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ)
    make = environment.get('MAKE') or shutil.which('make')
    overrides = []
    if os.name == 'nt':
        toolchain = pathlib.Path.home() / '.platformio/packages' \
            / 'toolchain-atmelavr/bin'
        mingw = ROOT / 'out/test-toolchain/mingw64/bin'
        git_shell = pathlib.Path('C:/Program Files/Git/usr/bin')
        environment['PATH'] = os.pathsep.join(
            (str(toolchain), str(mingw), str(git_shell), environment['PATH']))
        make = make or str(mingw / 'mingw32-make.exe')
        overrides = ['SHELL=C:/Progra~1/Git/usr/bin/sh.exe',
                     'PYTHON=' + pathlib.Path(sys.executable).as_posix()]
    make = make or 'make'
    config_text = (ROOT / 'test/configs/atmega328p-dht.config').read_text()
    profiles = [('disabled', config_text.replace(
        'CONFIG_WANT_DHT=y', '# CONFIG_WANT_DHT is not set'), False),
                ('enabled', config_text, True)]
    for name in ('atmega328', 'atmega2560'):
        profiles.append((name, (ROOT / ('test/configs/%s.config' % name))
                         .read_text(), False))
    for name in ('current-switch', 'peltier-controller'):
        text = (ROOT / ('mcu-build-config-' + name)).read_text()
        # Keep device identifiers out of the generated firmware dictionaries.
        text = re.sub(r'^CONFIG_USB_SERIAL_NUMBER=.*$',
                      'CONFIG_USB_SERIAL_NUMBER="12345"', text, flags=re.M)
        profiles.append((name, text, False))
        profiles.append((name + '-dht', text + '\nCONFIG_WANT_DHT=y\n', True))
    builds = {}
    for label, text, enabled in profiles:
        config = OUTPUT / (label + '.config')
        config.write_text(text)
        out = 'out/dht-build/%s/' % (label,)
        common = [make, 'OUT=' + out, 'KCONFIG_CONFIG=' + config.as_posix()]
        common += overrides
        if os.name == 'nt':
            # Git for Windows emulates directory symlinks by copying. Prepare
            # equivalent include directories after Kconfig, avoiding make's
            # repeated unlink of these directories (no admin privileges needed).
            config_env = dict(environment, KCONFIG_CONFIG=str(config))
            run([sys.executable, 'lib/kconfiglib/olddefconfig.py',
                 'src/Kconfig'],
                config_env, OUTPUT / (label + '-config.log'))
            for name in ('src/avr', 'src/generic'):
                (ROOT / out / name).mkdir(parents=True, exist_ok=True)
            for source, destination in (('avr', 'board'),
                                        ('generic', 'board-generic/board')):
                directory = ROOT / out / destination
                directory.mkdir(parents=True, exist_ok=True)
                for header in (ROOT / 'src' / source).glob('*.h'):
                    shutil.copy2(header, directory / header.name)
            (ROOT / out / 'board-link').write_text('# Windows include copies\n')
        else:
            run(common + ['olddefconfig'], environment,
                OUTPUT / (label + '-config.log'))
        run(common + ['-j2', 'all'], environment,
            OUTPUT / (label + '-build.log'))
        elf = ROOT / out / 'klipper.elf'
        size = run(['avr-size', str(elf)], environment)
        print(label + ':\n' + size)
        sizes = list(map(int, size.splitlines()[1].split()[:3]))
        symbols = run(['avr-nm', '-S', '--size-sort', str(elf)], environment)
        disassembly = run(['avr-objdump', '-d', str(elf)], environment)
        (OUTPUT / (label + '-symbols.txt')).write_text(symbols)
        (OUTPUT / (label + '-disassembly.txt')).write_text(disassembly)
        dictionary = json.loads((ROOT / out / 'klipper.dict').read_text())
        formats = list(dictionary['commands']) + list(dictionary['responses'])
        has_dht = any(key.startswith('config_dht ') for key in formats)
        assert has_dht == enabled
        assert not any(key.startswith(('query_dht_diagnostics ',
                                       'query_dht_timing ', 'set_dht_trace ',
                                       'dht_diagnostics ', 'dht_timing '))
                       for key in formats)
        assert not any(name in symbols for name in (
            'timer_probe', 'trace_requested', 'dht_gpio_timer',
            'dht_frame_diagnostics', 'active_dht'))
        if enabled:
            assert dictionary['config']['DHT_VERSION'] == 1
            assert re.search(r'\bT __vector_1\b', symbols)
            assert re.search(r'\bT __vector_3\b', symbols)
            bounds = irq_cycle_bound(disassembly)
            assert max(bounds.values()) < 26 * 16
            (OUTPUT / (label + '-irq-cycles.json')).write_text(
                json.dumps(bounds, indent=2) + '\n')
            print('IRQ cycle upper bounds: ' + str(bounds))
        else:
            assert not re.search(r'\bT __vector_[13]\b', symbols)
        builds[label] = dict(text=sizes[0], data=sizes[1], bss=sizes[2])
    before, after = builds['disabled'], builds['enabled']
    builds['delta'] = {key: after[key] - before[key] for key in before}
    assert builds['delta']['text'] + builds['delta']['data'] <= 4096
    (OUTPUT / 'sizes.json').write_text(json.dumps(builds, indent=2) + '\n')
    print(json.dumps(builds, indent=2))


if __name__ == '__main__':
    main()
