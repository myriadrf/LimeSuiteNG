# LimeSDR-Micro board tests

| Script | Purpose |
|---|---|
| `rf_test.py` | manufacturing RF test: the RF steps of `limeOEM --test` |

It uses the shared tooling in `../../equipment/`. See its README for the instrument drivers,
the station config and the exit code and output conventions.

```sh
# all instruments manual: the operator is told what to set and types in the readings
scripts/boards/limesdr-micro/rf_test.py -o results.csv

# signal generator over SCPI, spectrum analyzer manual, tools from a build tree
scripts/boards/limesdr-micro/rf_test.py --bindir build/bin --siggen tcp://192.168.1.20 -o results.csv

# station config file with instruments, fixture and cable losses
scripts/boards/limesdr-micro/rf_test.py --station station.json -o results.csv --json "sn_$(date +%s).json"
```

`--captures DIR` keeps the Rx recordings (SigMF). `--tests rx` or `--tests tx` runs one group.

## RF test

DUT ports: `RX_A`, `RX_B`, `TX` (the names used for `path_loss_db` in the station config).

The steps and limits match `LimeSDR_Micro::RFTest` (`src/boards/LimeSDR_Micro/LimeSDR_Micro.cpp`):

| Test | DUT port | Setup | Pass |
|---|---|---|---|
| `LNA_H/L/W A` | `RX_A` | generator 1202 MHz, -60 dBm. Rx LO 1200 MHz, 40 MS/s, gain 30 | peak at +2 MHz ±50 kHz, level ±4 dB of -32/-38/-37 dBFS |
| `LNA_H/L/W B` | `RX_B` | same | ±4 dB of -35/-39/-39 dBFS |
| `Tx Band1/2` | `TX` | Tx LO 2100 MHz, gain 52, VSPA tone +5 MHz at amplitude 0.9 | ≥ -18 dBm at 2105 MHz |

The Rx level comes from a `limeTRX` capture. It is analysed the same way as limeOEM (one
8192-point FFT, no window, DC bin skipped), so the dBFS limits are the same. The Tx tone comes
from `limeConfig --txtone`.

To change limits or settings without editing the script, pass `--limits` with a JSON file. Its
`rx` and `tx` keys apply to a whole group, and a test name applies to that one test:

```json
{ "tx": { "min_dbm": -17.0, "max_dbm": -10.0 }, "LNA_H B": { "expected_dbfs": -34.0 } }
```

Not covered: limeOEM also writes the default VCTCXO DAC value to the EEPROM when it was never
set. That is not an RF measurement and stays with `limeOEM`.

## Tx tone generator

`limeConfig --txtone <Hz>` transmits a tone at the given offset from the Tx LO. Use 0 for DC,
and the `=` form for negative offsets: `--txtone=-1e6`. `--txtoneamplitude` sets the amplitude
(0 to 1, default 0.9). `--txtone off` stops the tone. The tone is set after any other
configuration in the same call, and it keeps running after limeConfig exits:

```sh
limeConfig -d Micro --chmask 0x1 --txtone 5e6 --txtoneamplitude 0.5
limeConfig -d Micro --chmask 0x1 --txtone off
```

On the LimeSDR-Micro this is the VSPA tone of the single Tx pipeline (channel 0), because the
LA9310 does the data conversion. The LMS7002M test signal generator does not reach the RF ports.
