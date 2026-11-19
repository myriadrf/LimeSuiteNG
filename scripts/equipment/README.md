# equipment: shared tooling for board manufacturing tests

Python package (3.8+, numpy) used by the board specific test scripts in `../boards/`. It holds
everything that is not tied to one board: instrument control, the operator interface, the
LimeSuiteNG CLI tool wrappers, capture analysis and result reporting.

| Module | Purpose |
|---|---|
| `instruments.py` | generic `SignalGenerator` / `SpectrumAnalyzer` interfaces, manual and SCPI drivers |
| `scpi.py` | SCPI transports: `tcp://host[:port]`, `usbtmc:///dev/usbtmcN`, `visa://<resource>` (pyvisa) |
| `fixture.py` | DUT port connections: manual, none, or a shell command |
| `operator.py` | operator prompts: instructions, typed-in measurements, `q` to abort |
| `station.py` | station config: instruments, fixture, cable path losses |
| `limetools.py` | `limeDevice` / `limeConfig` / `limeTRX` wrappers |
| `dsp.py` | SigMF capture reading, FFT peak search (same method as limeOEM) |
| `report.py` | results, console summary, CSV row per board, JSON details |

A board script puts the parent directory on `sys.path` and imports `equipment.*` (see
`../boards/limesdr-micro/rf_test.py`).

## Instruments and the operator

Test scripts only use the generic interfaces:

- `SignalGenerator`: `set_cw(frequency_hz, power_dbm)`, `set_output(enabled)`
- `SpectrumAnalyzer`: `setup(center_hz, span_hz, rbw_hz, ref_level_dbm)`, `measure_peak(expected_hz)`
- `Fixture`: `connect(dut_port, instrument)`

Each one has two kinds of driver:

- **manual** (the default): every call becomes an instruction for the operator
  ("Set the signal generator to CW 1.202 GHz, -60.00 dBm and turn the RF output ON") and waits
  for ENTER. Measurements ask the operator to type the reading. Repeated settings are only
  shown once. Typing `q` at any prompt aborts the run.
- **scpi**: the instrument is controlled remotely. The default commands are the common
  Keysight/R&S/Rigol/Siglent style. Any command can be replaced in the station config
  (`commands`, see the `DEFAULT_COMMANDS` of each driver). A command that ends with `?` is a
  query. A command whose `{parameter}` is not given is skipped.

On the command line of a board script, an instrument is given as `manual` or as a SCPI resource
string (`--siggen tcp://192.168.1.20`).

The fixture is `manual` (the operator moves the cables), `none` (permanently wired), or
`command`, which runs a shell command such as an RF switch matrix driver:
`{"driver": "command", "connect": "rfswitch {dut_port} {instrument}"}`.

Adding a new instrument type means adding a driver class in `instruments.py` and a case in its
`create_*` factory. The board scripts do not change.

## Station config

A JSON file passed to the board scripts with `--station` (example: `station.example.json`):

```json
{
  "signal_generator": { "driver": "scpi", "resource": "tcp://192.168.1.20:5025" },
  "spectrum_analyzer": { "driver": "manual" },
  "fixture": { "driver": "manual" },
  "path_loss_db": { "RX_A": 1.5, "RX_B": 1.5, "TX": 2.0 }
}
```

`path_loss_db` holds the loss of the cabling to each DUT port as a positive number. The signal
generator level is raised by it, and the loss is added to the spectrum analyzer readings, so
limits always apply at the DUT connector. The DUT port names are defined by each board script.

## Conventions for board scripts

- Exit code 0 if the board passed, 1 if it failed, and 2 if the run did not complete (tool or
  instrument error, or the operator aborted).
- `-o FILE` appends one CSV row per board (date, S/N, result, the value of each measurement).
  `--json FILE` writes all the details.
- `--bindir` selects the lime tools directory (default: `$LIMESUITENG_BIN` or `PATH`). `-v`
  traces every tool command and SCPI exchange.
