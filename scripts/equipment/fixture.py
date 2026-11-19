"""DUT port connections.

The test scripts ask the fixture to connect an instrument to a DUT port. The manual fixture asks the
operator to move the cable. The command fixture runs a user supplied shell command (e.g. driving an RF
switch matrix). The 'none' fixture assumes everything is wired permanently.
"""

import shlex
import subprocess


class Fixture:
    def __init__(self):
        self.connections = {}  # dut port -> instrument label

    def connect(self, dut_port, instrument):
        if self.connections.get(dut_port) == instrument:
            return
        # an instrument can be cabled to one DUT port at a time
        for port, inst in list(self.connections.items()):
            if inst == instrument:
                del self.connections[port]
        self._connect(dut_port, instrument)
        self.connections[dut_port] = instrument

    def release(self):
        """Called at the end of the run."""
        if self.connections:
            self._release()
        self.connections.clear()

    def _connect(self, dut_port, instrument):
        pass

    def _release(self):
        pass


class ManualFixture(Fixture):
    def __init__(self, operator):
        super().__init__()
        self.operator = operator

    def _connect(self, dut_port, instrument):
        self.operator.instruct("Connect the %s to the DUT %s port" % (instrument, dut_port))

    def _release(self):
        self.operator.info("Test finished, the DUT cables can be disconnected.")


class CommandFixture(Fixture):
    """Runs config 'connect' command template with {dut_port} and {instrument}, and optional 'release'."""

    def __init__(self, connect_command, release_command=None):
        super().__init__()
        self.connect_command = connect_command
        self.release_command = release_command

    def _connect(self, dut_port, instrument):
        command = self.connect_command.format(dut_port=shlex.quote(dut_port), instrument=shlex.quote(instrument))
        subprocess.run(command, shell=True, check=True)

    def _release(self):
        if self.release_command:
            subprocess.run(self.release_command, shell=True, check=True)


def create_fixture(spec, operator):
    if spec is None or spec == "manual":
        return ManualFixture(operator)
    if spec == "none":
        return Fixture()
    driver = spec.get("driver", "manual")
    if driver == "manual":
        return ManualFixture(operator)
    if driver == "none":
        return Fixture()
    if driver == "command":
        return CommandFixture(spec["connect"], spec.get("release"))
    raise ValueError("unknown fixture driver: " + driver)
