"""Test results collection and output."""

import csv
import datetime
import json
import os
import sys
from dataclasses import asdict, dataclass, field
from typing import Optional


@dataclass
class Result:
    name: str
    passed: bool
    value: Optional[float] = None
    unit: str = ""
    low: Optional[float] = None
    high: Optional[float] = None
    message: str = ""
    details: dict = field(default_factory=dict)

    def limits_text(self):
        if self.low is not None and self.high is not None:
            return "[%g, %g]" % (self.low, self.high)
        if self.low is not None:
            return ">= %g" % self.low
        if self.high is not None:
            return "<= %g" % self.high
        return ""


def check_limits(value, low=None, high=None):
    return value is not None and (low is None or value >= low) and (high is None or value <= high)


class Report:
    def __init__(self, test_name, serial=None, device=None):
        self.test_name = test_name
        self.serial = serial
        self.device = device
        self.started = datetime.datetime.now().astimezone()
        self.results = []
        self.error = None  # set when the run did not complete

    def add(self, result, out=sys.stderr):
        self.results.append(result)
        value = "%.2f %s" % (result.value, result.unit) if result.value is not None else ""
        line = "  [%s] %-12s %-14s %s" % ("PASS" if result.passed else "FAIL", result.name, value, result.limits_text())
        if result.message:
            line += "  " + result.message
        print(line, file=out, flush=True)

    @property
    def passed(self):
        return self.error is None and bool(self.results) and all(r.passed for r in self.results)

    def print_summary(self, out=sys.stderr):
        print("\n=== %s: S/N %s ===" % (self.test_name, self.serial), file=out)
        for r in self.results:
            value = "%.2f %s" % (r.value, r.unit) if r.value is not None else "-"
            print("  %-12s %-14s %-16s %s" % (r.name, value, r.limits_text(), "PASS" if r.passed else "FAIL"), file=out)
        if self.error:
            print("  ERROR: %s" % self.error, file=out)
        print("=== %s ===" % ("PASSED" if self.passed else "FAILED"), file=out, flush=True)

    def append_csv(self, path):
        """Appends one row per board: date, S/N, overall result, then each result's value."""
        row = {"Date": self.started.isoformat(timespec="seconds"), "S/N": self.serial,
               "Result": "PASS" if self.passed else "FAIL"}
        for r in self.results:
            row[r.name] = "%.2f" % r.value if r.value is not None else ("PASS" if r.passed else "FAIL")
        new_file = not os.path.exists(path) or os.path.getsize(path) == 0
        if not new_file:
            with open(path, newline="") as f:
                header = next(csv.reader(f), [])
            # keep the existing column order; extra columns are appended
            fieldnames = header + [k for k in row if k not in header]
            if fieldnames != header:
                print("Warning: %s has different columns, appending with a new header" % path, file=sys.stderr)
                new_file = True
        else:
            fieldnames = list(row)
        with open(path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if new_file:
                writer.writeheader()
            writer.writerow(row)

    def write_json(self, path):
        data = {
            "test": self.test_name,
            "serial": self.serial,
            "device": self.device,
            "started": self.started.isoformat(timespec="seconds"),
            "passed": self.passed,
            "error": self.error,
            "results": [asdict(r) for r in self.results],
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2)
