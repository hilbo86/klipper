"""Check firmware capability restrictions with Klipper's real Kconfig."""
import os
import pathlib
import sys
import tempfile
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'lib/kconfiglib'))
import kconfiglib


class TestDHTKconfig(unittest.TestCase):
    def load_config(self, text):
        with tempfile.TemporaryDirectory() as directory:
            config = pathlib.Path(directory) / '.config'
            config.write_text(text)
            with mock.patch.dict(os.environ, {'srctree': str(ROOT)}):
                kconfig = kconfiglib.Kconfig(str(ROOT / 'src/Kconfig'),
                                             warn=False)
                kconfig.load_config(str(config))
            return kconfig

    def test_only_supported_build_can_enable_dht(self):
        text = (ROOT / 'test/configs/atmega328p-dht.config').read_text()
        self.assertEqual(self.load_config(text).syms['WANT_DHT'].str_value, 'y')
        variants = [
            text.replace('CONFIG_WANT_DHT=y', ''),
            text.replace('# CONFIG_HAVE_STEPPER is not set',
                         'CONFIG_HAVE_STEPPER=y'),
            text.replace('# CONFIG_WANT_HARD_PWM is not set',
                         'CONFIG_WANT_HARD_PWM=y'),
            text.replace('atmega328p', 'atmega328'),
            text.replace('atmega328p', 'atmega2560'),
            text.replace('atmega328p', 'lgt8f328p'),
        ]
        for frequency in (8000000, 20000000):
            variants.append('CONFIG_LOW_LEVEL_OPTIONS=y\n' + text.replace(
                'CONFIG_AVR_FREQ_16000000=y',
                'CONFIG_AVR_FREQ_%d=y' % frequency))
        for index, variant in enumerate(variants):
            with self.subTest(variant=index):
                self.assertEqual(self.load_config(variant)
                                 .syms['WANT_DHT'].str_value, 'n')


if __name__ == '__main__':
    unittest.main()
