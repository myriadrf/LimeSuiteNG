"""SCPI transports: raw TCP socket, Linux USBTMC device file, or VISA (if pyvisa is installed).

Resource strings:
    tcp://192.168.1.20[:5025]       raw SCPI socket (default port 5025)
    usbtmc:///dev/usbtmc0           Linux usbtmc kernel driver
    visa://TCPIP0::host::INSTR      any VISA resource (requires pyvisa)
"""

import os
import socket


class ScpiError(Exception):
    pass


class TcpTransport:
    def __init__(self, host, port=5025, timeout=10.0):
        self.address = (host, port)
        self.sock = socket.create_connection(self.address, timeout=timeout)
        self.sock.settimeout(timeout)
        self.buffer = b""

    def write(self, command):
        self.sock.sendall(command.encode("ascii") + b"\n")

    def read(self):
        while b"\n" not in self.buffer:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ScpiError("connection closed by instrument %s:%d" % self.address)
            self.buffer += chunk
        line, self.buffer = self.buffer.split(b"\n", 1)
        return line.decode("ascii", errors="replace").strip()

    def close(self):
        self.sock.close()


class UsbTmcTransport:
    def __init__(self, path, timeout=10.0):
        self.path = path
        self.fd = os.open(path, os.O_RDWR)

    def write(self, command):
        os.write(self.fd, command.encode("ascii") + b"\n")

    def read(self):
        # the usbtmc driver applies its own read timeout and fails with ETIMEDOUT
        try:
            return os.read(self.fd, 65536).decode("ascii", errors="replace").strip()
        except OSError as e:
            raise ScpiError("read from %s failed: %s" % (self.path, e)) from e

    def close(self):
        os.close(self.fd)


class VisaTransport:
    def __init__(self, resource, timeout=10.0):
        try:
            import pyvisa
        except ImportError as e:
            raise ScpiError("visa:// resources require the pyvisa package (pip install pyvisa pyvisa-py)") from e
        self.rm = pyvisa.ResourceManager()
        self.inst = self.rm.open_resource(resource)
        self.inst.timeout = int(timeout * 1000)
        self.inst.read_termination = "\n"
        self.inst.write_termination = "\n"

    def write(self, command):
        self.inst.write(command)

    def read(self):
        return self.inst.read().strip()

    def close(self):
        self.inst.close()


def open_transport(resource, timeout=10.0):
    if resource.startswith("tcp://"):
        address = resource[len("tcp://"):]
        host, _, port = address.partition(":")
        return TcpTransport(host, int(port) if port else 5025, timeout)
    if resource.startswith("usbtmc://"):
        return UsbTmcTransport(resource[len("usbtmc://"):], timeout)
    if resource.startswith("visa://"):
        return VisaTransport(resource[len("visa://"):], timeout)
    raise ScpiError("unknown SCPI resource '%s', expected tcp://, usbtmc:// or visa://" % resource)


class ScpiInstrument:
    """Thin SCPI session with command logging."""

    def __init__(self, resource, timeout=10.0, log=None):
        self.resource = resource
        self.transport = open_transport(resource, timeout)
        self.log = log

    def write(self, command):
        if self.log:
            self.log("SCPI %s <- %s" % (self.resource, command))
        self.transport.write(command)

    def query(self, command):
        self.write(command)
        reply = self.transport.read()
        if self.log:
            self.log("SCPI %s -> %s" % (self.resource, reply))
        return reply

    def query_float(self, command):
        reply = self.query(command)
        try:
            return float(reply.split(",")[0])
        except ValueError as e:
            raise ScpiError("unexpected reply to %s: '%s'" % (command, reply)) from e

    def close(self):
        self.transport.close()
