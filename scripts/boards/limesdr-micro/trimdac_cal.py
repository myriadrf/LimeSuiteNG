#!/usr/bin/env python3
"""LimeSDR-Micro reference oscillator (VCTCXO) trim DAC calibration.

All the board clocks, the LMS7002M Tx LO and the LA9310 sample clock, come from the VCTCXO, so its
frequency error is measured on the Tx output: the VSPA tone generator (limeConfig --txtone) transmits
a CW tone at a small offset from the Tx LO, and a spectrum analyzer locked to an accurate reference
measures its frequency. A DC tone cannot be used, the baseband path does not pass DC to the RF output.

The trim DAC is set with limeConfig --trimdac, which does not store it. The search:
1. measure the error at the start value (the current DAC value by default);
2. measure again one probe step away, which gives the DAC slope (ppm per LSB);
3. step to the value the slope predicts and measure, refining the slope from the last two points
   (secant method), until the error is within the tolerance.

The DAC response is always checked: if the frequency does not change over the probe step, it is
measured at the far end of the DAC range too, and no change there fails the board with "trim DAC
might be defective".

The best value stays set when the calibration passes, otherwise the start value is restored. With
--store, a passing value is also written to the EEPROM (limeConfig --storetrimdac), which the board
loads at power up, and read back.

The analyzer is controlled over SCPI, or by the operator following the printed instructions (the
default). See README.md here and ../../equipment/README.md.
"""

import argparse
import json
import os
import sys
import time
import traceback

# shared test tooling: scripts/equipment
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))

from equipment.instruments import format_hz  # noqa: E402
from equipment.limetools import LimeTools, ToolError  # noqa: E402
from equipment.operator import Operator, OperatorAbort  # noqa: E402
from equipment.report import Report, Result, check_limits  # noqa: E402
from equipment.station import Station  # noqa: E402

# Calibration settings, each entry can be overridden with --limits
PLAN = {
    "port": "TX",
    "sample_rate": 40e6,
    "lo": 1800e6,
    "path": "Band2",
    "gain": 52,
    "tone_offset": 1e6,  # small, so that the measurement is the LO error even if the baseband clock differed
    "tone_amplitude": 0.9,
    "search_span": 100e3,  # the signal is searched within +/- span/2 of the nominal frequency: +/-27 ppm
    "resolution_hz": 10.0,  # frequency measurement resolution
    "tolerance_ppm": 0.1,  # final error limit
    "max_initial_ppm": None,  # limit for the error before the calibration, None: no limit
    "min_slope_ppb": None,  # lowest acceptable |DAC slope| in ppb/LSB (too little pull range), None: no limit
    "start_dac": None,  # None: the current DAC value
    "probe_step": 2000,  # DAC step used to measure the slope
    "min_dac": 0,
    "max_dac": 65535,
    "max_iterations": 8,  # measurements after the slope probe
    "settle_s": 0.5,  # wait after changing the DAC
}


def build_plan(limits_path, overrides):
    plan = dict(PLAN)
    if limits_path:
        with open(limits_path) as f:
            values = json.load(f)
        unknown = set(values) - set(plan)
        if unknown:
            raise ValueError("unknown settings in %s: %s" % (limits_path, ", ".join(sorted(unknown))))
        plan.update(values)
    plan.update({k: v for k, v in overrides.items() if v is not None})
    return plan


class TrimDacCalibration:
    def __init__(self, tools, station, operator, report, plan, store=False):
        self.tools = tools
        self.station = station
        self.operator = operator
        self.report = report
        self.plan = plan
        self.expected = plan["lo"] + plan["tone_offset"]
        self.points = []  # (dac, frequency, error ppm) in measurement order
        self.tone_active = False
        self.store = store  # store the calibrated value to the EEPROM
        self.response = ""  # why the DAC is considered defective

    def run(self):
        p = self.plan
        self.operator.info("\n--- Trim DAC calibration ---")
        self.tools.ppsdo_disable()  # a running PPSDO loop would change the DAC while measuring
        initial_dac = self.tools.get_trimdac()
        start = int(p["start_dac"]) if p["start_dac"] is not None else initial_dac
        if not p["min_dac"] <= start <= p["max_dac"]:
            raise ValueError("start DAC value %d is outside of [%d, %d]" % (start, p["min_dac"], p["max_dac"]))
        self.operator.info("Trim DAC: %d, starting from %d" % (initial_dac, start))

        specan = self.station.spectrum_analyzer
        self.station.fixture.connect(p["port"], specan.label)
        self.start_tone()
        best, slope = None, None
        try:
            best, message, slope = self.search(specan, start)
        finally:
            self.stop_tone()
            slope_ok = slope is not None and check_limits(abs(slope * 1e3), p["min_slope_ppb"])
            converged = best is not None and abs(best[2]) <= p["tolerance_ppm"]
            passed = converged and slope_ok
            final_dac = best[0] if passed else initial_dac
            try:
                self.tools.set_trimdac(final_dac)
            except ToolError as e:
                self.operator.info("Warning: failed to set the trim DAC to %d: %s" % (final_dac, e))

        first = self.points[0]
        max_initial = p["max_initial_ppm"]
        self.report.add(Result("Initial error", check_limits(abs(first[2]), None, max_initial), first[2], "ppm",
                               -max_initial if max_initial is not None else None, max_initial,
                               "DAC %d @ %s" % (first[0], format_hz(first[1])), fmt="%+.3f"))

        response = self.response
        if slope is None:
            self.operator.info("ERROR: %s" % response)
        self.report.add(Result("DAC slope", slope_ok, slope * 1e3 if slope is not None else None, "ppb/LSB",
                               p["min_slope_ppb"], message="" if slope is not None else response, fmt="%.4f"))

        tolerance = p["tolerance_ppm"]
        self.report.add(Result("Final error", converged, best[2], "ppm", -tolerance, tolerance,
                               message or "@ %s" % format_hz(best[1]), fmt="%+.3f"))

        stored = None
        if passed and self.store:
            self.tools.store_trimdac(best[0])
            stored = self.tools.get_stored_trimdac()
        details = {"points": [{"dac": d, "frequency_hz": f, "error_ppm": e} for d, f, e in self.points],
                   "initial_dac": initial_dac, "stored_dac": stored, "config": p}
        if not passed:
            note = "restored to %d" % initial_dac
        elif stored is not None:
            note = "stored" if stored == best[0] else "store failed: reads back %d" % stored
        else:
            note = "left set (not stored)"
        self.report.add(Result("Trim DAC", passed and (stored is None or stored == best[0]), best[0], "",
                               p["min_dac"], p["max_dac"], note, details, fmt="%.0f"))
        if passed:
            self.operator.info("Calibrated trim DAC value: %d (slope %.4f ppb/LSB), %s" % (best[0], slope * 1e3, note))

    def search(self, specan, start):
        """Returns the best point (dac, frequency, error ppm), a failure message or None if it converged, and
        the DAC slope in ppm/LSB, None if the DAC has no effect on the frequency (self.response says why)."""
        p = self.plan
        tolerance = p["tolerance_ppm"]
        noise_ppm = 5 * p["resolution_hz"] / self.expected * 1e6  # smaller changes are measurement noise
        dac, error = start, self.measure(specan, start)

        # The second point gives the slope. It is measured even if the start is within the tolerance,
        # to check that the DAC works.
        step = int(p["probe_step"])
        probe = dac + step if dac + step <= p["max_dac"] else dac - step
        probe = min(max(probe, p["min_dac"]), p["max_dac"])
        probe_error = self.measure(specan, probe)
        if abs(probe_error - error) < noise_ppm:
            # no change over the probe step: try the far end of the range before giving up on the DAC
            far = p["max_dac"] if p["max_dac"] - dac >= dac - p["min_dac"] else p["min_dac"]
            probe, probe_error = far, self.measure(specan, far)
            if abs(probe_error - error) < noise_ppm:
                self.response = ("trim DAC might be defective: the frequency changed %+.3f ppm between DAC %d and %d"
                                 % (probe_error - error, dac, probe))
                return self.best(), "the trim DAC does not change the frequency", None
        slope = (probe_error - error) / (probe - dac)  # ppm per LSB
        self.response = ""
        dac, error = (probe, probe_error) if abs(probe_error) < abs(error) else (dac, error)

        for _ in range(int(p["max_iterations"])):
            if abs(error) <= tolerance:
                return self.best(), None, slope
            target = int(round(dac - error / slope))
            clamped = min(max(target, p["min_dac"]), p["max_dac"])
            if clamped == dac:
                if clamped != target:
                    return self.best(), "out of the DAC range: needs %d" % target, slope
                return self.best(), "the DAC resolution is not enough for the tolerance", slope
            new_error = self.measure(specan, clamped)
            # refine the slope from the last two points if they are far enough apart to be accurate
            if abs(new_error - error) >= noise_ppm:
                slope = (new_error - error) / (clamped - dac)
            dac, error = clamped, new_error
            if clamped != target and abs(error) > tolerance:
                return self.best(), "out of the DAC range: needs %d" % target, slope
        if abs(error) <= tolerance:
            return self.best(), None, slope
        return self.best(), "no convergence in %d steps" % p["max_iterations"], slope

    def measure(self, specan, dac):
        p = self.plan
        self.tools.set_trimdac(dac)
        time.sleep(p["settle_s"])
        frequency = specan.measure_frequency(self.expected, p["search_span"], p["resolution_hz"])
        error = (frequency - self.expected) / self.expected * 1e6
        self.points.append((dac, frequency, error))
        self.operator.info("  DAC %5d: %.1f Hz, error %+.3f ppm" % (dac, frequency, error))
        return error

    def best(self):
        return min(self.points, key=lambda point: abs(point[2]))

    def start_tone(self):
        p = self.plan
        self.tools.config(initialize=True, samplerate=p["sample_rate"], rxen=0, txen=1, txlo=p["lo"], rxlo=p["lo"],
                          txpath=p["path"], txgain=p["gain"])
        self.tools.config(chmask=0x2, txen=0)  # single Tx channel
        self.tone_active = True
        self.tools.config(chmask=0x1, txtone=p["tone_offset"], txtoneamplitude=p["tone_amplitude"])

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
    parser.add_argument("--station", help="station config JSON: instruments, fixture")
    parser.add_argument("--specan", help="spectrum analyzer: 'manual' or a SCPI resource (tcp://host[:port], "
                                         "usbtmc:///dev/usbtmc0, visa://...). Overrides the station config")
    parser.add_argument("--fixture", choices=["manual", "none"], help="cable connections: by the operator, or "
                                                                     "permanently wired. Overrides the station config")
    parser.add_argument("--limits", help="JSON with calibration setting overrides, see PLAN in the script")
    parser.add_argument("--start-dac", type=lambda v: int(v, 0), help="DAC value to start from (default: the current value)")
    parser.add_argument("--tolerance", type=float, help="final error limit in ppm (default: %g)" % PLAN["tolerance_ppm"])
    parser.add_argument("--store", action="store_true", help="store the calibrated value to the EEPROM when the "
                                                               "calibration passes (default: only set it until power off)")
    parser.add_argument("-o", "--output", help="CSV file to append the results row to")
    parser.add_argument("--json", help="write the detailed results to this JSON file")
    parser.add_argument("-l", "--log", help="lime tools log level (error, warning, info, verbose, debug)")
    parser.add_argument("-v", "--verbose", action="store_true", help="print the tool commands, output and SCPI traffic")
    args = parser.parse_args()

    operator = Operator()
    log = (lambda text: print("    " + text.replace("\n", "\n    "), file=sys.stderr)) if args.verbose else None
    tools = LimeTools(args.device, args.bindir, args.log, log)
    plan = build_plan(args.limits, {"start_dac": args.start_dac, "tolerance_ppm": args.tolerance})

    try:
        info = tools.device_info()
    except ToolError as e:
        print("ERROR: %s" % e, file=sys.stderr)
        return 2
    serial = info.get("Serial number", "unknown")
    report = Report("LimeSDR-Micro trim DAC calibration", serial, info.get("handle"))
    operator.info("Device: %s" % info.get("handle"))
    operator.info("Serial number: %s" % serial)

    station = None
    try:
        overrides = {"spectrum_analyzer": args.specan, "fixture": args.fixture}
        station = Station.load(args.station, operator, overrides, log)
        TrimDacCalibration(tools, station, operator, report, plan, args.store).run()
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
