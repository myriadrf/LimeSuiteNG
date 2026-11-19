"""Test station configuration: instruments, fixture and RF path losses.

The configuration is a JSON file (see station.example.json). Command line options override it.
"""

import json

from .fixture import create_fixture
from .instruments import create_signal_generator, create_spectrum_analyzer


class Station:
    def __init__(self, config, operator, log=None):
        self.config = config
        self.operator = operator
        self.log = log
        self.path_loss_db = dict(config.get("path_loss_db", {}))
        self._siggen = None
        self._specan = None
        self.fixture = create_fixture(config.get("fixture"), operator)

    @classmethod
    def load(cls, path, operator, overrides=None, log=None):
        config = {}
        if path:
            with open(path) as f:
                config = json.load(f)
        for key, value in (overrides or {}).items():
            if value is not None:
                config[key] = value
        return cls(config, operator, log)

    # instruments are connected on first use, so a test that does not need one runs without it

    @property
    def signal_generator(self):
        if self._siggen is None:
            self._siggen = create_signal_generator(self.config.get("signal_generator"), self.operator, self.log)
        return self._siggen

    @property
    def spectrum_analyzer(self):
        if self._specan is None:
            self._specan = create_spectrum_analyzer(self.config.get("spectrum_analyzer"), self.operator, self.log)
        return self._specan

    def loss(self, dut_port):
        """Loss in dB of the cabling between the instrument and the DUT port (positive number)."""
        return float(self.path_loss_db.get(dut_port, 0.0))

    def close(self):
        for inst in (self._siggen, self._specan):
            if inst:
                try:
                    inst.close()
                except Exception as e:  # noqa: BLE001 - closing must not hide the test result
                    self.operator.info("Warning: failed to close instrument: %s" % e)
