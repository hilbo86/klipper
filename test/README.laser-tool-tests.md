# Laser tool tests

The tool manager's unit tests and host doubles are owned by Klipper:

- `test_laser_tool_manager.py`: ID stability and freshness, file preflight,
  output interlocks, shutdown, movement limits, pause/resume admission and
  calibration lifecycle. Includes median filtering, hysteresis, motor-noise
  regression cases, reacquisition and immediate raw-value fault handling.
- `laser_tool_harness.py`: synthetic host, ADC, pin, fan and motion interfaces.

From the Klipper checkout, run:

```sh
python -m unittest discover -s test -p test_laser_tool_manager.py
```

These tests need only Python's standard library. They do not read or copy
printer configuration files and do not require Jinja2 or a neighbouring
configuration checkout. Macro callbacks are external host interfaces in this
suite; the printer's actual macro behavior is tested separately.

ID filter options, status fields and timing are described in
[`docs/Laser_Tool_Manager.md`](../docs/Laser_Tool_Manager.md).

The `Klipper_config` repository owns the integration tests for its compressor,
job, power and parking macros, and the tests for its Y-Max measurement script.
Those tests load the real configuration and use the shared doubles from this
Klipper checkout. Their command, from `Klipper_config`, is:

```sh
python -m unittest discover -s tests
```

The macro integration tests require Jinja2 and the sibling checkout layout
`klipper/` and `Klipper_config/`. Frontend tests remain in Mainsail.
