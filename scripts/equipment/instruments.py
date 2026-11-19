"""Generic RF instrument interfaces used by the board tests.

Test scripts only use the abstract SignalGenerator / SpectrumAnalyzer methods. Each interface has:
- a SCPI driver, for remotely controllable instruments. The command set defaults to the common
  Keysight/R&S/Rigol/Siglent style and every command can be overridden in the station config;
- a manual driver, which turns each call into instructions for the operator and asks the operator
  to type in the measured values. It is used when the instrument cannot be automated.
"""

import string
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

from .scpi import ScpiInstrument


def format_hz(hz):
    if hz is None:
        return "-"
    for div, unit in ((1e9, "GHz"), (1e6, "MHz"), (1e3, "kHz")):
        if abs(hz) >= div:
            return "%.6g %s" % (hz / div, unit)
    return "%g Hz" % hz


@dataclass
class PeakMeasurement:
    power_dbm: float
    frequency_hz: Optional[float] = None  # None if the instrument/operator did not report it


class SignalGenerator(ABC):
    label = "signal generator"

    @abstractmethod
    def set_cw(self, frequency_hz, power_dbm):
        """Sets a continuous wave output frequency and level. Does not change the output state."""

    @abstractmethod
    def set_output(self, enabled):
        """Turns the RF output on or off."""

    def close(self):
        pass


class SpectrumAnalyzer(ABC):
    label = "spectrum analyzer"

    @abstractmethod
    def setup(self, center_hz, span_hz, rbw_hz=None, ref_level_dbm=None):
        """Sets the displayed frequency range. None leaves a setting unchanged."""

    @abstractmethod
    def measure_peak(self, expected_hz):
        """Measures the strongest signal in the span, expected near expected_hz. Returns PeakMeasurement."""

    def close(self):
        pass


# --- Manual (operator instructed) drivers ---------------------------------------------------------


class ManualSignalGenerator(SignalGenerator):
    def __init__(self, operator):
        self.operator = operator
        self.cw = None  # requested (frequency, power)
        self.applied_cw = None  # what the operator was last told to set
        self.output = False

    def _cw_text(self):
        return "CW %s, %.2f dBm" % (format_hz(self.cw[0]), self.cw[1])

    def set_cw(self, frequency_hz, power_dbm):
        self.cw = (frequency_hz, power_dbm)
        if self.output and self.cw != self.applied_cw:
            self.operator.instruct("Change the %s output to %s" % (self.label, self._cw_text()))
            self.applied_cw = self.cw

    def set_output(self, enabled):
        if enabled == self.output and (not enabled or self.cw == self.applied_cw):
            return
        if enabled:
            self.operator.instruct("Set the %s to %s and turn the RF output ON" % (self.label, self._cw_text()))
            self.applied_cw = self.cw
        else:
            self.operator.instruct("Turn the %s RF output OFF" % self.label)
        self.output = enabled


class ManualSpectrumAnalyzer(SpectrumAnalyzer):
    def __init__(self, operator):
        self.operator = operator
        self.settings = {}
        self.shown_settings = None

    def setup(self, center_hz, span_hz, rbw_hz=None, ref_level_dbm=None):
        for key, value in (("center", center_hz), ("span", span_hz), ("rbw", rbw_hz), ("ref", ref_level_dbm)):
            if value is not None:
                self.settings[key] = value

    def measure_peak(self, expected_hz):
        if self.settings != self.shown_settings:
            parts = ["center %s" % format_hz(self.settings.get("center")), "span %s" % format_hz(self.settings.get("span"))]
            if "rbw" in self.settings:
                parts.append("RBW %s" % format_hz(self.settings["rbw"]))
            if "ref" in self.settings:
                parts.append("reference level %.1f dBm" % self.settings["ref"])
            self.operator.instruct("Set the %s: %s" % (self.label, ", ".join(parts)))
            self.shown_settings = dict(self.settings)
        power = self.operator.ask_float(
            "Place a peak marker on the signal at %s and read its level" % format_hz(expected_hz), "dBm")
        return PeakMeasurement(power_dbm=power, frequency_hz=None)


# --- SCPI drivers ----------------------------------------------------------------------------------


def _render(templates, **params):
    """Formats command templates. A command that references a parameter which is None is skipped."""
    if isinstance(templates, str):
        templates = [templates]
    commands = []
    for template in templates:
        fields = [f for _, f, _, _ in string.Formatter().parse(template) if f]
        names = [f.split(".")[0].split("[")[0].split(":")[0] for f in fields]
        if any(params.get(n) is None for n in names):
            continue
        commands.append(template.format(**params))
    return commands


class _ScpiDriver:
    DEFAULT_COMMANDS = {}

    def __init__(self, resource, commands=None, timeout=10.0, log=None):
        self.commands = dict(self.DEFAULT_COMMANDS)
        self.commands.update(commands or {})
        self.inst = ScpiInstrument(resource, timeout, log)
        self.idn = self.inst.query("*IDN?")
        self._run("init")

    def _run(self, name, **params):
        """Runs a command list. Commands ending with '?' are queries; the last reply is returned."""
        reply = None
        for command in _render(self.commands.get(name, []), **params):
            if command.rstrip().endswith("?"):
                reply = self.inst.query(command)
            else:
                self.inst.write(command)
        return reply

    def close(self):
        self.inst.close()


class ScpiSignalGenerator(_ScpiDriver, SignalGenerator):
    DEFAULT_COMMANDS = {
        "init": [],
        "set_cw": [":FREQ {frequency_hz:.0f}", ":POW {power_dbm:.2f}", "*OPC?"],
        "output_on": [":OUTP ON", "*OPC?"],
        "output_off": [":OUTP OFF", "*OPC?"],
    }

    def set_cw(self, frequency_hz, power_dbm):
        self._run("set_cw", frequency_hz=frequency_hz, power_dbm=power_dbm)

    def set_output(self, enabled):
        self._run("output_on" if enabled else "output_off")


class ScpiSpectrumAnalyzer(_ScpiDriver, SpectrumAnalyzer):
    DEFAULT_COMMANDS = {
        "init": [":INIT:CONT OFF"],
        "setup": [
            ":FREQ:CENT {center_hz:.0f}",
            ":FREQ:SPAN {span_hz:.0f}",
            ":BAND {rbw_hz:.0f}",
            ":DISP:WIND:TRAC:Y:RLEV {ref_level_dbm:.1f}",
            "*OPC?",
        ],
        "sweep": [":INIT:IMM", "*OPC?"],
        "peak": [":CALC:MARK1:STAT ON", ":CALC:MARK1:MAX"],
        "marker_frequency": [":CALC:MARK1:X?"],
        "marker_power": [":CALC:MARK1:Y?"],
    }

    def setup(self, center_hz, span_hz, rbw_hz=None, ref_level_dbm=None):
        self._run("setup", center_hz=center_hz, span_hz=span_hz, rbw_hz=rbw_hz, ref_level_dbm=ref_level_dbm)

    def measure_peak(self, expected_hz):
        self._run("sweep")
        self._run("peak")
        frequency = float(self._run("marker_frequency").split(",")[0])
        power = float(self._run("marker_power").split(",")[0])
        return PeakMeasurement(power_dbm=power, frequency_hz=frequency)


# --- Factory ---------------------------------------------------------------------------------------


def _normalize(spec):
    """Accepts a station config dict, or a shorthand string: 'manual' or a SCPI resource string."""
    if spec is None:
        return {"driver": "manual"}
    if isinstance(spec, str):
        return {"driver": "manual"} if spec == "manual" else {"driver": "scpi", "resource": spec}
    return spec


def create_signal_generator(spec, operator, log=None):
    spec = _normalize(spec)
    driver = spec.get("driver", "manual")
    if driver == "manual":
        return ManualSignalGenerator(operator)
    if driver == "scpi":
        return ScpiSignalGenerator(spec["resource"], spec.get("commands"), spec.get("timeout", 10.0), log)
    raise ValueError("unknown signal generator driver: " + driver)


def create_spectrum_analyzer(spec, operator, log=None):
    spec = _normalize(spec)
    driver = spec.get("driver", "manual")
    if driver == "manual":
        return ManualSpectrumAnalyzer(operator)
    if driver == "scpi":
        return ScpiSpectrumAnalyzer(spec["resource"], spec.get("commands"), spec.get("timeout", 10.0), log)
    raise ValueError("unknown spectrum analyzer driver: " + driver)
