# Laser tool tests

The tool manager's unit tests and host doubles are owned by Klipper:

- `test_laser_tool_manager.py`: ID stability and freshness, file preflight,
  output interlocks, shutdown, movement limits, pause/resume admission and
  calibration lifecycle. Includes moving means, isolated outlier rejection,
  hysteresis, motor-noise regression cases, confirmation of real ID loss without
  waiting for the mean window, and full reacquisition.
- `laser_tool_harness.py`: synthetic host, ADC, pin, fan and motion interfaces.
- `test_laser_tool_bounds.py`: power-off measurement sessions, one-axis steps,
  temporary Cartesian limits, interlocks, isolated ID spikes during movement,
  confirmed ID loss and bounds before offset calibration.

From the Klipper checkout, run:

```sh
python -m unittest discover -s test -p 'test_laser_tool*.py'
```

These tests need only Python's standard library. They do not read or copy
printer configuration files and do not require Jinja2 or a neighbouring
configuration checkout. Macro callbacks are external host interfaces in this
suite; the printer's actual macro behavior is tested separately.

ID filter options, status fields and timing are described in
[`docs/Laser_Tool_Manager.md`](../docs/Laser_Tool_Manager.md).

The `Klipper_config` repository owns the integration tests for its compressor,
job, power and parking macros, and the tests for its X/Y/Z measurement script.
Those tests load the real configuration and use the shared doubles from this
Klipper checkout. Their command, from `Klipper_config`, is:

```sh
python -m unittest discover -s tests
```

The macro integration tests require Jinja2 and the sibling checkout layout
`klipper/` and `Klipper_config/`. Frontend tests remain in Mainsail.
