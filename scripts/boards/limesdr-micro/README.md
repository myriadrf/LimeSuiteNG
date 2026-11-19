# LimeSDR-Micro board tests

| Script | Purpose |
|---|---|
| `rf_test.py` | manufacturing RF test: the RF steps of `limeOEM --test` |
| `trimdac_cal.py` | reference oscillator (VCTCXO) trim DAC calibration from the Tx frequency |

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

## Trim DAC calibration

```sh
# spectrum analyzer over SCPI, locked to a GPS/rubidium 10 MHz reference
scripts/boards/limesdr-micro/trimdac_cal.py --specan tcp://192.168.1.30 --fixture none --store -o cal.csv
```

All the board clocks (the LMS7002M LO and the LA9310 sample clock) come from the VCTCXO. The
script transmits the VSPA tone at Tx LO 1800 MHz + 1 MHz from `TX` (Band2) and measures its
frequency with the spectrum analyzer. A tone at DC cannot be used, because the baseband path does
not pass DC to the RF output. The 1 MHz offset is small enough that the result is the LO error.

1. The PPSDO is disabled (`limePPSDO --disable`), so that it does not move the DAC.
2. The error is measured at the current DAC value (`--start-dac` to start elsewhere), then one
   probe step (2000 LSB) away. This gives the slope in ppm/LSB. It is measured even when the
   start value is already within the tolerance, to check that the DAC works. If the frequency does
   not change over the probe step, it is also measured at the far end of the DAC range. No change
   there either fails the board with "trim DAC might be defective".
3. The script steps to the value the slope predicts, measures, and refines the slope from the
   last two points until the error is within `--tolerance` (default 0.1 ppm).

The results are the initial error, the DAC slope, the final error and the DAC value
(`Initial error`, `DAC slope`, `Final error`, `Trim DAC` columns). The JSON has every measured
point. On a pass the calibrated value stays set. On a failure (defective DAC, out of the DAC
range, no convergence) the start value is restored.

The value is set with `limeConfig --trimdac`, which does not store it, so it is lost at power
off. With `--store`, a passing value is also written to the EEPROM with
`limeConfig --storetrimdac`. The board loads it from there at power up. limeConfig reads the
stored value back to verify it, and the script reads it once more for the report.

The other settings (LO, offset, gain, search span, resolution, probe step, iteration count, an
optional `max_initial_ppm` limit, and an optional `min_slope_ppb` limit that fails a DAC with too
little pull range) are in `PLAN` in the script and can be overridden with
`--limits`, e.g. `{ "lo": 1000e6, "resolution_hz": 1 }`.

## Trim DAC

`limeConfig --trimdac <value>` sets the reference oscillator trim DAC without storing it
(decimal or 0x hex, 0 to 65535 on the LimeSDR-Micro). `limeConfig --trimdac get` prints the
current value, and `limeConfig --trimdac stored` prints the value stored in the non-volatile
memory (the EEPROM `XO_DAC` region on the LimeSDR-Micro). `--storetrimdac` stores the value
given with `--trimdac`, or the current one, and verifies it:

```sh
limeConfig -d Micro --trimdac 46993 --storetrimdac
limeConfig -d Micro --trimdac stored
```
 The value is applied after any other configuration in the same call, and
`--initialize` does not change it. A running PPSDO loop overrides it.

limeConfig finds the DAC through the `TrimDAC` node of the device tree (`lime::ITrimDAC`,
`src/include/limesuiteng/ITrimDAC.h`), so the options work on every board that lists one.

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
