#!/usr/bin/env python3
"""LimeSDR-Micro manufacturing RF test.

Runs the RF steps of `limeOEM --test` (LimeSDR_Micro::RFTest in src/boards/LimeSDR_Micro/LimeSDR_Micro.cpp)
with the LimeSuiteNG CLI tools:

Rx: a signal generator feeds a CW tone at 1202 MHz, -60 dBm into the Rx A and Rx B ports. For each
    LNA path the board is tuned to 1200 MHz, limeTRX captures samples, and the received peak must
    be at +2 MHz with the expected dBFS level.
Tx: the VSPA tone generator (limeConfig --txtone) transmits LO + 5 MHz on 2100 MHz through Tx Band1
    and Band2. A spectrum analyzer on the Tx port measures the level.

Instruments are controlled over SCPI, or by the operator following the printed instructions when they
cannot be automated (the default). See README.md here and ../../equipment/README.md.
"""

import argparse
import json
import os
import sys
import traceback

# shared test tooling: scripts/equipment
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

from equipment.dsp import find_peak  # noqa: E402
from equipment.instruments import format_hz  # noqa: E402
from equipment.limetools import LimeTools, ToolError  # noqa: E402
from equipment.operator import Operator, OperatorAbort  # noqa: E402
from equipment.report import Report, Result, check_limits  # noqa: E402
from equipment.station import Station  # noqa: E402

# Test plan, values from LimeSDR_Micro::RFTest. Each test's entries can be overridden with --limits.
RX_COMMON = {
    "sample_rate": 40e6,
    "lo": 1200e6,
    "offset": 2e6,  # signal generator frequency offset from the LO
    "gain": 30,
    "generator_dbm": -60.0,  # level at the DUT port, the station path loss is added
    "tolerance_db": 4.0,
    "tolerance_hz": 50e3,
    "fft_size": 8192,
}
RX_TESTS = [
    {"name": "LNA_H A", "port": "RX_A", "channel": 0, "path": "LNAH", "expected_dbfs": -32.0},
    {"name": "LNA_L A", "port": "RX_A", "channel": 0, "path": "LNAL", "expected_dbfs": -38.0},
    {"name": "LNA_W A", "port": "RX_A", "channel": 0, "path": "LNAW", "expected_dbfs": -37.0},
    {"name": "LNA_H B", "port": "RX_B", "channel": 1, "path": "LNAH", "expected_dbfs": -35.0},
    {"name": "LNA_L B", "port": "RX_B", "channel": 1, "path": "LNAL", "expected_dbfs": -39.0},
    {"name": "LNA_W B", "port": "RX_B", "channel": 1, "path": "LNAW", "expected_dbfs": -39.0},
]

TX_COMMON = {
    "sample_rate": 40e6,
    "lo": 2100e6,
    "tone_offset": 40e6 / 8,  # limeOEM uses 1/8 of the sample rate
    "tone_amplitude": 0.9,
    "gain": 52,
    "rx_path": "LNAW",
    "rx_gain": 30,
    "min_dbm": -18.0,  # level at the DUT port, the station path loss is added to the measurement
    "max_dbm": None,
    "tolerance_hz": 50e3,  # checked only if the analyzer reports the peak frequency
    "span": 2e6,
    "rbw": 10e3,
    "ref_level_dbm": 0.0,
}
TX_TESTS = [
    {"name": "Tx Band1", "port": "TX", "path": "Band1"},
    {"name": "Tx Band2", "port": "TX", "path": "Band2"},
]


def build_plan(limits_path):
    rx = [dict(RX_COMMON, **t) for t in RX_TESTS]
    tx = [dict(TX_COMMON, **t) for t in TX_TESTS]
    if limits_path:
        with open(limits_path) as f:
            overrides = json.load(f)
        known = {t["name"] for t in rx + tx} | {"rx", "tx"}
        unknown = set(overrides) - known
        if unknown:
            raise ValueError("unknown test names in %s: %s" % (limits_path, ", ".join(sorted(unknown))))
        for group, tests in (("rx", rx), ("tx", tx)):
            for test in tests:
                test.update(overrides.get(group, {}))
                test.update(overrides.get(test["name"], {}))
    return rx, tx


class MicroRFTest:
    def __init__(self, tools, station, operator, report, capture_dir=None):
        self.tools = tools
        self.station = station
        self.operator = operator
        self.report = report
        self.capture_dir = capture_dir
        self.tone_active = False

    def run_rx(self, tests):
        self.operator.info("\n--- Rx tests ---")
        siggen = self.station.signal_generator
        try:
            for test in tests:
                siggen.set_cw(test["lo"] + test["offset"], test["generator_dbm"] + self.station.loss(test["port"]))
                self.station.fixture.connect(test["port"], siggen.label)
                siggen.set_output(True)
                self.report.add(self.rx_test(test))
        finally:
            siggen.set_output(False)

    def rx_test(self, t):
        self.tools.config(initialize=True, samplerate=t["sample_rate"], rxen=1, txen=0, rxlo=t["lo"],
                          txlo=t["lo"] + t["offset"], rxpath=t["path"], rxgain=t["gain"])
        fft_size = int(t["fft_size"])
        capture = None
        if self.capture_dir:
            capture = os.path.join(self.capture_dir, "%s_%s" % (self.report.serial, t["name"].replace(" ", "_")))
        # like limeOEM, skip the first block in case the digital DC corrector is still settling
        iq, sample_rate = self.tools.capture(t["channel"], 2 * fft_size, capture)
        frequency, dbfs = find_peak(iq, sample_rate, fft_size, skip=fft_size)

        low, high = t["expected_dbfs"] - t["tolerance_db"], t["expected_dbfs"] + t["tolerance_db"]
        frequency_ok = abs(frequency - t["offset"]) < t["tolerance_hz"]
        passed = check_limits(dbfs, low, high) and frequency_ok
        message = "@ %s" % format_hz(frequency)
        if not frequency_ok:
            message += " (expected %s +/- %s)" % (format_hz(t["offset"]), format_hz(t["tolerance_hz"]))
        return Result(t["name"], passed, dbfs, "dBFS", low, high, message,
                      {"peak_offset_hz": frequency, "sample_rate": sample_rate, "config": t})

    def run_tx(self, tests):
        self.operator.info("\n--- Tx tests ---")
        specan = self.station.spectrum_analyzer
        try:
            for test in tests:
                self.station.fixture.connect(test["port"], specan.label)
                specan.setup(test["lo"] + test["tone_offset"], test["span"], test["rbw"], test["ref_level_dbm"])
                self.report.add(self.tx_test(test, specan))
        finally:
            self.stop_tone()

    def tx_test(self, t, specan):
        self.tools.config(initialize=True, samplerate=t["sample_rate"], rxen=1, txen=1, rxlo=t["lo"], txlo=t["lo"],
                          rxpath=t["rx_path"], rxgain=t["rx_gain"], txpath=t["path"], txgain=t["gain"])
        self.tools.config(chmask=0x2, txen=0)  # single Tx channel, like limeOEM
        self.tone_active = True
        self.tools.config(chmask=0x1, txtone=t["tone_offset"], txtoneamplitude=t["tone_amplitude"])
        try:
            expected = t["lo"] + t["tone_offset"]
            peak = specan.measure_peak(expected)
        finally:
            self.stop_tone()

        level = peak.power_dbm + self.station.loss(t["port"])
        passed = check_limits(level, t["min_dbm"], t["max_dbm"])
        message = ""
        if peak.frequency_hz is not None:
            message = "@ %s" % format_hz(peak.frequency_hz)
            if abs(peak.frequency_hz - expected) > t["tolerance_hz"]:
                passed = False
                message += " (expected %s +/- %s)" % (format_hz(expected), format_hz(t["tolerance_hz"]))
        return Result(t["name"], passed, level, "dBm", t["min_dbm"], t["max_dbm"], message,
                      {"measured_dbm": peak.power_dbm, "path_loss_db": self.station.loss(t["port"]),
                       "peak_frequency_hz": peak.frequency_hz, "config": t})

    def stop_tone(self):
        if not self.tone_active:
            return
        try:
            self.tools.config(chmask=0x1, txtone="off")
            self.tools.config(txen=0)
            self.tone_active = False
        except ToolError as e:
            self.operator.info("Warning: failed to stop the Tx tone: %s" % e)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-d", "--device", default="Micro", help="device filter for the lime tools (default: %(default)s)")
    parser.add_argument("--bindir", help="directory of the lime CLI tools (default: $LIMESUITENG_BIN or PATH)")
    parser.add_argument("--station", help="station config JSON: instruments, fixture, path losses")
    parser.add_argument("--siggen", help="signal generator: 'manual' or a SCPI resource (tcp://host[:port], "
                                         "usbtmc:///dev/usbtmc0, visa://...). Overrides the station config")
    parser.add_argument("--specan", help="spectrum analyzer: 'manual' or a SCPI resource. Overrides the station config")
    parser.add_argument("--fixture", choices=["manual", "none"], help="cable connections: by the operator, or "
                                                                     "permanently wired. Overrides the station config")
    parser.add_argument("--limits", help="JSON with test plan overrides: {\"rx\": {...}, \"tx\": {...}, \"<test name>\": {...}}")
    parser.add_argument("--tests", default="rx,tx", help="test groups to run (default: %(default)s)")
    parser.add_argument("-o", "--output", help="CSV file to append the results row to")
    parser.add_argument("--json", help="write the detailed results to this JSON file")
    parser.add_argument("--captures", help="keep the Rx captures (SigMF) in this directory")
    parser.add_argument("-l", "--log", help="lime tools log level (error, warning, info, verbose, debug)")
    parser.add_argument("-v", "--verbose", action="store_true", help="print the tool commands, output and SCPI traffic")
    args = parser.parse_args()

    groups = {g.strip() for g in args.tests.split(",") if g.strip()}
    if not groups or not groups <= {"rx", "tx"}:
        parser.error("--tests takes a list of: rx, tx")

    operator = Operator()
    log = (lambda text: print("    " + text.replace("\n", "\n    "), file=sys.stderr)) if args.verbose else None
    tools = LimeTools(args.device, args.bindir, args.log, log)
    rx_plan, tx_plan = build_plan(args.limits)
    if args.captures:
        os.makedirs(args.captures, exist_ok=True)

    try:
        info = tools.device_info()
    except ToolError as e:
        print("ERROR: %s" % e, file=sys.stderr)
        return 2
    serial = info.get("Serial number", "unknown")
    report = Report("LimeSDR-Micro RF test", serial, info.get("handle"))
    operator.info("Device: %s" % info.get("handle"))
    operator.info("Serial number: %s" % serial)

    station = None
    try:
        overrides = {"signal_generator": args.siggen, "spectrum_analyzer": args.specan, "fixture": args.fixture}
        station = Station.load(args.station, operator, overrides, log)
        test = MicroRFTest(tools, station, operator, report, args.captures)
        if "rx" in groups:
            test.run_rx(rx_plan)
        if "tx" in groups:
            test.run_tx(tx_plan)
        station.fixture.release()
    except KeyboardInterrupt:
        report.error = "interrupted"
    except OperatorAbort as e:
        report.error = str(e)
    except Exception as e:  # noqa: BLE001 - any failure must still produce a report
        report.error = "%s: %s" % (type(e).__name__, e)
        if args.verbose:
            traceback.print_exc()
    finally:
        if station:
            station.close()

    report.print_summary()
    if args.output:
        report.append_csv(args.output)
    if args.json:
        report.write_json(args.json)
    if report.error:
        return 2
    return 0 if report.passed else 1


if __name__ == "__main__":
    sys.exit(main())
