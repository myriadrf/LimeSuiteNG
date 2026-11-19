"""Wrappers around the LimeSuiteNG command line tools."""

import os
import re
import shutil
import subprocess
import tempfile

from . import dsp


class ToolError(Exception):
    pass


def _number(value):
    """Formats a number for the command line without losing precision."""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float) and value.is_integer():
        return str(int(value))
    return str(value)


class LimeTools:
    def __init__(self, device="", bindir=None, log_level=None, log=None):
        """
        device:    -d device filter passed to the tools (e.g. "Micro"), empty to use the only connected device
        bindir:    directory of the tools, default: $LIMESUITENG_BIN or PATH
        log_level: tools log level (error, warning, info, verbose, debug)
        log:       callable receiving command lines and tool output, for verbose tracing
        """
        self.device = device
        self.bindir = bindir or os.environ.get("LIMESUITENG_BIN")
        self.log_level = log_level
        self.log = log

    def _tool(self, name):
        if self.bindir:
            path = os.path.join(self.bindir, name)
            if not os.access(path, os.X_OK):
                raise ToolError("%s not found in %s" % (name, self.bindir))
            return path
        path = shutil.which(name)
        if not path:
            raise ToolError("%s not found in PATH, use --bindir" % name)
        return path

    def run(self, name, args, timeout=60, device=True):
        command = [self._tool(name)]
        if device and self.device:
            command += ["-d", self.device]
        if self.log_level:
            command += ["-l", self.log_level]
        command += [str(a) for a in args]
        if self.log:
            self.log("$ " + " ".join(command))
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=timeout)
        except subprocess.TimeoutExpired as e:
            raise ToolError("%s timed out after %d s" % (name, timeout)) from e
        output = (result.stdout + result.stderr).strip()
        if self.log and output:
            self.log(output)
        if result.returncode != 0:
            raise ToolError("%s failed (exit code %d): %s" % (name, result.returncode, output[-2000:]))
        return output

    # --- limeDevice ---

    def device_info(self):
        """Returns the limeDevice -f fields of the selected device as a dict (e.g. 'Serial number')."""
        output = self.run("limeDevice", ["-f"], device=False)
        blocks = re.split(r"^(?=\d+: )", output, flags=re.MULTILINE)
        blocks = [b for b in blocks if re.match(r"\d+: ", b)]
        if self.device:
            blocks = [b for b in blocks if self.device.lower() in b.splitlines()[0].lower()]
        if not blocks:
            raise ToolError("no device found" + (" matching '%s'" % self.device if self.device else ""))
        if len(blocks) > 1:
            raise ToolError("multiple devices found, select one with --device:\n" +
                            "\n".join(b.splitlines()[0] for b in blocks))
        lines = blocks[0].splitlines()
        info = {"handle": lines[0].split(": ", 1)[1]}
        for line in lines[1:]:
            match = re.match(r"\s+([^:]+?)\s*: (.*)$", line)
            if match:
                info[match.group(1)] = match.group(2).strip()
        return info

    # --- limeConfig ---

    def config(self, initialize=False, chmask=None, **params):
        """Runs limeConfig. Keyword arguments map to the long options: rxlo=1e9 -> --rxlo 1000000000.
        Values of None are skipped, True flags (e.g. rxcalibrate=True) are passed without a value."""
        args = []
        if initialize:
            args.append("--initialize")
        if chmask is not None:
            args += ["--chmask", hex(chmask)]
        for key, value in params.items():
            if value is None:
                continue
            if value is True and key.endswith("calibrate"):
                args.append("--" + key)
            elif isinstance(value, (int, float)) and not isinstance(value, bool) and value < 0:
                args.append("--%s=%s" % (key, _number(value)))  # keep negative numbers from parsing as flags
            else:
                args += ["--" + key, _number(value)]
        return self.run("limeConfig", args, timeout=120)

    # --- limeTRX ---

    def capture(self, channel, samples, output_base=None, timeout=30):
        """Receives samples from one Rx channel. Returns (samples as complex64 ndarray, sample rate)."""
        temp_dir = None
        if output_base is None:
            temp_dir = tempfile.TemporaryDirectory(prefix="limetrx-capture-")
            output_base = os.path.join(temp_dir.name, "capture")
        try:
            chunk = min(samples, 8192)
            self.run("limeTRX", [
                "--channelsmask=%x" % (1 << channel),
                "--linkFormat", "I16",
                "--fftSize", str(chunk),
                "-s", str(samples),
                "-o", output_base,
            ], timeout=timeout)
            iq, sample_rate = dsp.read_sigmf(output_base)
            if len(iq) < samples:
                raise ToolError("limeTRX captured %d samples, expected %d" % (len(iq), samples))
            return iq, sample_rate
        finally:
            if temp_dir:
                temp_dir.cleanup()
