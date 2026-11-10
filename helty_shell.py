#!/usr/bin/env python3
"""HeltyShell - interactive shell for a Helty Flow40 ventilation unit over Modbus RTU.

Protocol: Helty Device Remote Protocol (HDRP) and Helty Device Application
Protocol (HDAP) over RS-485, 19200 baud 8N1. Modbus functions used: 03 (read
holding registers), 04 (read input registers), 06 (write single holding register).

Standard library only: the serial port is driven through termios, so the script
runs with the system Python on macOS and Linux.

Usage:
    ./helty                      interactive shell (port and unit found automatically)
    ./helty --port /dev/cu.xxx   shell on a specific port
    ./helty status               runs one command and exits (0 on success)

Write safeguards (details in HeltyShell.safe_write and in the 'safety' command):
    - only HR 1000 (machine state) is writable, with values 0-7; other registers
      only with --expert and a typed confirmation; HR 0-15 never
    - model check (PID in IR 1013) before writing
    - no write if the register already holds the requested value
    - minimum interval between two writes, also across separate runs
    - the bus is listened to before writing and the write is blocked if another
      master is heard; this is best effort (a master that polls rarely goes unnoticed)
    - explicit confirmation to start the unit while a fault is active
    - no automatic retry when the write is not acknowledged
    - read-back after every write and a log file
    - no broadcast; read-only mode available
    - serial port opened exclusively
"""

from __future__ import annotations

import argparse
import atexit
import cmd
import contextlib
import errno
import fcntl
import glob
import inspect
import json
import os
import select
import signal
import struct
import sys
import termios
import time
from datetime import datetime
from typing import Callable, Dict, List, Optional, Tuple

# -----------------------------------------------------------------------------
# Protocol constants (HDRP / HDAP)
# -----------------------------------------------------------------------------

DEFAULT_BAUD = 19200
DEFAULT_PARITY = "N"
DEFAULT_SLAVE = 2              # address tried when none is remembered for the port
DEFAULT_TIMEOUT = 0.5          # s; the specification says 100 ms, the margin covers USB latency
FRAME_GAP = 0.02               # s of minimum silence between two frames (3.5 characters at 19200 baud ≈ 2 ms)
SCAN_TIMEOUT = 0.15            # s to wait for each address during a scan

HR_STATUS = 1000               # machine state holding register (values in MODES)
IR_FIRST = 1000                # first application input register
IR_T_AMB = 1000                # indoor temperature, signed tenths of °C
IR_T_EXT = 1001                # outdoor temperature, signed tenths of °C
IR_ALARMS = 1006               # alarm bitmask (bits in ALARM_BITS)
IR_FW = 1012                   # firmware version
IR_PID = 1013                  # product ID of the device

# Values of HR_STATUS
MODES: Dict[int, str] = {
    0: "Off",
    1: "Speed 1",
    2: "Speed 2",
    3: "Speed 3",
    4: "Speed 4",
    5: "Hyper",
    6: "Night",
    7: "Free cooling",
}

# Names accepted by 'mode' and as standalone commands, with the matching HR_STATUS value
MODE_ALIASES: Dict[str, int] = {
    "off": 0,
    "v1": 1, "v2": 2, "v3": 3, "v4": 4,
    "hyper": 5, "boost": 5,
    "night": 6,
    "freecooling": 7, "free": 7, "fc": 7,
}

# The Flow40 has no humidity, CO2 or VOC sensors: IR 1006 always has the
# "humidity sensor missing" and "CO2 sensor missing" bits set (0x5000) and IR 1015
# (VOC) reads 0. These bits are not alarms; 'status' reports it if they clear.
EXPECTED_FLAGS = 0x1000 | 0x4000

# Bits of IR 1006: (mask, description, True = alarm / False = information only)
ALARM_BITS: List[Tuple[int, str, bool]] = [
    (0x0001, "Internal sensor fault", True),
    (0x0002, "External sensor fault", True),
    (0x0004, "Internal fan fault", True),
    (0x0008, "External fan fault", True),
    (0x0010, "EEPROM error", True),
    (0x0020, "Frost alarm", True),
    (0x0040, "High humidity (requires humidity sensor)", True),
    (0x0080, "Low humidity (requires humidity sensor)", True),
    (0x0100, "Temperature delta", True),
    (0x0200, "CO2 alarm (requires CO2 sensor)", True),
    (0x0400, "Filter alarm", True),
    (0x1000, "Humidity sensor missing (expected on the Flow40)", False),
    (0x4000, "CO2 sensor missing (expected on the Flow40)", False),
]

# Faults for which starting the unit requires an explicit confirmation
FAULT_MASK = 0x0001 | 0x0002 | 0x0004 | 0x0008 | 0x0010 | 0x0020

# Descriptions shown by 'dump' and 'read'; entries with "unknown" or "?" are guesses
IR_LABELS: Dict[int, str] = {
    0: "Customer ID",
    1: "PID",
    2: "HW revision",
    3: "FW revision",
    1000: "Indoor temperature (×0.1 °C)",
    1001: "Outdoor temperature (×0.1 °C)",
    1004: "unknown (400: maybe CO2 minimum)",
    1005: "CO2 (ppm) - no sensor on the Flow40",
    1006: "Alarms (bitmask)",
    1010: "unknown (maybe an hour counter)",
    1011: "temperature? (×0.1 °C)",
    1012: "Firmware version",
    1013: "PID",
    1014: "unknown",
    1015: "VOC (ppm) - no sensor on the Flow40",
    1020: "unknown (same as 1010)",
}

HR_LABELS: Dict[int, str] = {
    0: "Password (protected)",
    1: "Password confirmation (protected)",
    1000: "Machine state",
}
for _a in range(2, 16):
    HR_LABELS[_a] = "non-volatile data (protected)"

# Standard Modbus exception codes
MODBUS_EXCEPTIONS: Dict[int, str] = {
    1: "illegal function",
    2: "illegal data address",
    3: "illegal data value",
    4: "slave device failure",
    5: "acknowledge, processing in progress",
    6: "slave device busy",
}

# USB serial devices searched by find_ports, in order (macOS, then Linux)
PORT_PATTERNS = [
    "/dev/cu.usbserial*",
    "/dev/cu.usbmodem*",
    "/dev/cu.wchusbserial*",
    "/dev/ttyUSB*",
    "/dev/ttyACM*",
]

# -----------------------------------------------------------------------------
# Safeguards and persistent state
# -----------------------------------------------------------------------------

KNOWN_PIDS: Dict[int, str] = {0x1102: "Helty Flow40"}   # PIDs that writes are allowed to
PROTECTED_HR = range(0, 16)          # password and non-volatile data: never written
WRITE_RULES: Dict[int, Tuple[int, int]] = {HR_STATUS: (0, 7)}   # register -> (min, max) without --expert
WRITE_COOLDOWN = 3.0                 # s minimum between two writes, also across separate runs
BUS_LISTEN_OPEN = 0.5                # s of bus listening when the port is opened (warning only)
BUS_LISTEN_WRITE = 0.3               # s of bus listening before each write (blocks)
PID_ATTEMPTS = 2                     # reads of IR 1013 before the PID is considered unreadable

# The shell's files live next to the script.
# STATE_FILE (JSON): "port" last port where the unit was found, "slaves" {port: address},
# "last_write" epoch of the last write.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STATE_FILE = os.path.join(SCRIPT_DIR, ".helty_state.json")
WRITE_LOG = os.path.join(SCRIPT_DIR, "helty_writes.log")
HISTORY_FILE = os.path.join(SCRIPT_DIR, ".helty_history")


def load_state() -> dict:
    """Reads STATE_FILE.

    Returns the state dictionary, or {} if the file is missing, is not valid JSON
    or does not contain an object.
    """
    try:
        with open(STATE_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def save_state(state: dict) -> bool:
    """Writes state to STATE_FILE atomically (temporary file and os.replace).

    Returns False if the write fails; never raises.
    """
    try:
        tmp = STATE_FILE + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(state, f)
        os.replace(tmp, STATE_FILE)
        return True
    except OSError:
        return False

# -----------------------------------------------------------------------------
# Output formatting
# -----------------------------------------------------------------------------

# ANSI colours only when stdout is a terminal and the NO_COLOR variable is not set
_COLOR = sys.stdout.isatty() and "NO_COLOR" not in os.environ


def _c(code: str, s: object) -> str:
    """Wraps s in the ANSI sequence code when colours are on, otherwise returns str(s)."""
    return f"\033[{code}m{s}\033[0m" if _COLOR else str(s)


def green(s: object) -> str:
    """Green text, for confirmations."""
    return _c("32", s)


def red(s: object) -> str:
    """Red text, for errors and alarms."""
    return _c("31", s)


def yellow(s: object) -> str:
    """Yellow text, for warnings."""
    return _c("33", s)


def dim(s: object) -> str:
    """Dimmed text, for hints and secondary details."""
    return _c("2", s)


def bold(s: object) -> str:
    """Bold text."""
    return _c("1", s)


def hexdump(data: bytes) -> str:
    """Bytes as space-separated hex ("02 04 00 02"); "(empty)" if data is empty."""
    return " ".join(f"{b:02X}" for b in data) if data else "(empty)"


def signed16(v: int) -> int:
    """Interprets a 16-bit register as a signed integer (two's complement)."""
    return v - 0x10000 if v >= 0x8000 else v


def fmt_temp(raw: int) -> str:
    """Formats a temperature register (signed tenths of °C), for example "21.5 °C"."""
    return f"{signed16(raw) / 10:.1f} °C"


def fmt_s(v: float) -> str:
    """Formats a duration in seconds, for example "0.5 s"."""
    return f"{v:g} s"


def mode_name(v: int) -> str:
    """Mode name for an HR_STATUS value; "unknown (v)" if v is not in MODES."""
    return MODES.get(v, f"unknown ({v})")


def decode_alarms(v: int) -> Tuple[List[str], List[str]]:
    """Decodes the value of IR 1006.

    Returns (alarms, information): the descriptions of the active bits flagged as
    alarms and of those flagged as information in ALARM_BITS. Active bits that are
    not in ALARM_BITS are added to the alarms as "unknown bits".
    """
    alarms = [name for bit, name, real in ALARM_BITS if real and v & bit]
    infos = [name for bit, name, real in ALARM_BITS if not real and v & bit]
    known = 0
    for bit, _, _ in ALARM_BITS:
        known |= bit
    unknown = v & ~known & 0xFFFF
    if unknown:
        alarms.append(f"unknown bits 0x{unknown:04X}")
    return alarms, infos


def decode_pid(v: int) -> str:
    """Describes a PID, for example "0x1102 - Helty Flow40 (project 2, 2017)".

    The high byte is read as a two-digit year, the low byte as the project number.
    """
    year, project = v >> 8, v & 0xFF
    model = KNOWN_PIDS.get(v, "unknown model")
    return f"0x{v:04X} - {model} (project {project}, 20{year:02d})"


def parse_int(s: str) -> int:
    """Converts s to an integer, accepting the 0x, 0o and 0b prefixes. Raises ValueError."""
    return int(s, 0)


def ask(question: str) -> str:
    """Asks a question and returns the answer lowercased, without surrounding spaces.

    Ctrl+C, Ctrl+D or a closed stdin return "", which every confirmation treats as no.
    """
    try:
        return input(question).strip().lower()
    except (EOFError, KeyboardInterrupt):
        print(flush=True)
        return ""


@contextlib.contextmanager
def uninterruptible():
    """Ignores SIGINT (Ctrl+C) inside the block, then restores the previous handler.

    Used to finish the read-back and the log of a write that has already been sent.
    Must run on the main thread: on other threads signal.signal raises ValueError.
    """
    old = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        yield
    finally:
        signal.signal(signal.SIGINT, old)


# -----------------------------------------------------------------------------
# Modbus RTU
# -----------------------------------------------------------------------------


class ModbusError(Exception):
    """Modbus communication error: CRC, unexpected frame or port not available."""
    pass


class ModbusTimeout(ModbusError):
    """No response, or an incomplete response, within the timeout."""
    pass


class ModbusException(ModbusError):
    """Exception response from the slave (function code with bit 0x80 set); code is the Modbus code."""

    def __init__(self, code: int):
        """Builds the message from the description in MODBUS_EXCEPTIONS."""
        self.code = code
        desc = MODBUS_EXCEPTIONS.get(code, "unknown")
        super().__init__(f"the unit replied with exception {code:#04x}: {desc}")


class WriteRefused(Exception):
    """Write blocked by a safeguard; the message says which one."""


class Cancelled(Exception):
    """Operation cancelled by the user at a confirmation prompt; nothing was written."""


def crc16(data: bytes) -> int:
    """CRC-16/Modbus of data: reflected polynomial 0xA001, initial value 0xFFFF."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def with_crc(frame: bytes) -> bytes:
    """Returns frame followed by its CRC-16, low byte first as Modbus RTU requires."""
    c = crc16(frame)
    return frame + bytes((c & 0xFF, c >> 8))


# Supported baud rates -> termios constants, only those available on this platform
BAUD_CONSTANTS = {
    b: getattr(termios, f"B{b}")
    for b in (2400, 4800, 9600, 19200, 38400, 57600, 115200)
    if hasattr(termios, f"B{b}")
}


class SerialPort:
    """Raw serial port on a non-blocking file descriptor, driven through termios.

    8 data bits, 1 stop bit, configurable parity, exclusive access (see _lock).
    """

    def __init__(self, path: str, baud: int = DEFAULT_BAUD, parity: str = DEFAULT_PARITY):
        """Opens and configures path.

        Raises ValueError for an unsupported baud rate or parity, OSError if the port
        cannot be opened or is already in use (errno EBUSY). On error the file
        descriptor is closed.
        """
        if baud not in BAUD_CONSTANTS:
            raise ValueError(f"unsupported baud rate: {baud}")
        parity = parity.upper()
        if parity not in ("N", "E", "O"):
            raise ValueError("invalid parity: use N, E or O")
        self.path, self.baud, self.parity = path, baud, parity
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            self._lock()
            self._configure()
        except Exception:
            os.close(self.fd)
            raise

    def _lock(self) -> None:
        """Makes access exclusive: a second process on the same port would be a second master.

        flock excludes other instances of this shell; TIOCEXCL makes open() fail with
        EBUSY for any other program (macOS ignores it on pseudo-terminals). Raises
        OSError(EBUSY) if another instance already holds the lock.
        """
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)      # other instances of the shell
        except BlockingIOError:
            raise OSError(errno.EBUSY, os.strerror(errno.EBUSY)) from None
        try:
            fcntl.ioctl(self.fd, termios.TIOCEXCL)                  # every other program
        except (AttributeError, OSError):
            pass                     # driver without TIOCEXCL: only the lock between instances remains

    def _configure(self) -> None:
        """Sets raw mode, CS8, CREAD, CLOCAL, parity and baud rate, then flushes the buffers.

        VMIN = VTIME = 0: reads never block and timing is handled with select().
        """
        attrs = termios.tcgetattr(self.fd)
        cflag = termios.CS8 | termios.CREAD | termios.CLOCAL
        if self.parity == "E":
            cflag |= termios.PARENB
        elif self.parity == "O":
            cflag |= termios.PARENB | termios.PARODD
        cc = attrs[6]
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        speed = BAUD_CONSTANTS[self.baud]
        # [iflag, oflag, cflag, lflag, ispeed, ospeed, cc]: iflag, oflag and lflag at zero = raw
        termios.tcsetattr(self.fd, termios.TCSANOW, [0, 0, cflag, 0, speed, speed, cc])
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def describe(self) -> str:
        """Description for the user, for example "/dev/cu.usbserial-A10K · 19200 8N1"."""
        return f"{self.path} · {self.baud} 8{self.parity}1"

    def write(self, data: bytes) -> None:
        """Discards pending input bytes, sends data and waits until it has been transmitted (tcdrain)."""
        termios.tcflush(self.fd, termios.TCIFLUSH)   # leftovers of earlier frames or of other masters
        view = memoryview(data)
        while view:
            try:
                n = os.write(self.fd, view)
                view = view[n:]
            except BlockingIOError:
                select.select([], [self.fd], [], 0.1)
        termios.tcdrain(self.fd)

    def read_frame(self, expected_len: Callable[[bytearray], Optional[int]], timeout: float) -> bytes:
        """Reads one frame within timeout seconds.

        expected_len receives the bytes read so far and returns the total frame length,
        or None while it cannot be determined yet. Returns exactly that many bytes; any
        extra bytes stay in the buffer and are discarded by the next write().
        Raises ModbusTimeout ("no response" or "incomplete response").
        """
        buf = bytearray()
        deadline = time.monotonic() + timeout
        while True:
            need = expected_len(buf)
            if need is not None and len(buf) >= need:
                return bytes(buf[:need])
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                if buf:
                    raise ModbusTimeout(f"incomplete response: {hexdump(bytes(buf))}")
                raise ModbusTimeout("no response")
            ready, _, _ = select.select([self.fd], [], [], remaining)
            if not ready:
                continue
            try:
                chunk = os.read(self.fd, 256)
            except BlockingIOError:
                continue
            buf += chunk

    def listen(self, duration: float) -> bytes:
        """Listens to the bus for duration seconds without transmitting and returns the bytes received.

        The shell has sent no request, so any byte received comes from another master.
        """
        termios.tcflush(self.fd, termios.TCIFLUSH)
        buf = bytearray()
        deadline = time.monotonic() + duration
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return bytes(buf)
            ready, _, _ = select.select([self.fd], [], [], remaining)
            if ready:
                try:
                    buf += os.read(self.fd, 256)
                except BlockingIOError:
                    pass

    def close(self) -> None:
        """Releases TIOCEXCL and closes the file descriptor (the flock goes with it). Never raises."""
        try:
            # TIOCEXCL is cleared only on the last close of the device, which may not be this one
            fcntl.ioctl(self.fd, termios.TIOCNXCL)
        except (AttributeError, OSError):
            pass
        try:
            os.close(self.fd)
        except OSError:
            pass


def _expected_response_len(buf: bytearray) -> Optional[int]:
    """Expected total length of a Modbus RTU response, given its first bytes.

    Exception: 5 bytes. FC 03/04: 5 + byte count (third byte). FC 06/16: 8 bytes.
    Other function codes: 5, so the frame ends and is rejected by the later checks.
    Returns None while the bytes received are not enough to tell.
    """
    if len(buf) < 2:
        return None
    fc = buf[1]
    if fc & 0x80:
        return 5
    if fc in (3, 4):
        return None if len(buf) < 3 else 5 + buf[2]
    if fc in (6, 16):
        return 8
    return 5


class ModbusClient:
    """Modbus RTU client on a SerialPort: frame building, CRC and response validation."""

    def __init__(self, port: SerialPort, slave: int, timeout: float,
                 log: Optional[Callable[[str], None]] = None):
        """slave is the default address, timeout the maximum wait for a response in seconds.

        log, when given, receives one line for each frame sent and received.
        """
        self.port = port
        self.slave = slave
        self.timeout = timeout
        self.log = log
        self._last = 0.0          # time.monotonic() at the end of the last transaction

    def _transact(self, pdu: bytes, slave: int, timeout: Optional[float] = None) -> bytes:
        """Sends pdu to the slave and returns the response PDU (function code included).

        Keeps FRAME_GAP since the previous transaction. Raises ValueError for addresses
        outside 1-247, ModbusTimeout without a response, ModbusException for an
        exception response and ModbusError for a wrong CRC, sender or function code.
        """
        if not 1 <= slave <= 247:
            # no broadcast (0) and no reserved addresses: every request has exactly one recipient
            raise ValueError(f"invalid slave address: {slave} (allowed 1-247)")
        request = with_crc(bytes([slave]) + pdu)
        wait = FRAME_GAP - (time.monotonic() - self._last)
        if wait > 0:
            time.sleep(wait)
        if self.log:
            self.log(dim(f"TX → {hexdump(request)}"))
        try:
            self.port.write(request)
            try:
                response = self.port.read_frame(_expected_response_len,
                                                self.timeout if timeout is None else timeout)
            except ModbusTimeout as e:
                if self.log:
                    self.log(dim(f"RX ← {e}"))
                raise
        finally:
            self._last = time.monotonic()
        if self.log:
            self.log(dim(f"RX ← {hexdump(response)}"))
        body, crc = response[:-2], response[-2] | (response[-1] << 8)
        if crc16(body) != crc:
            raise ModbusError(f"wrong CRC in response: {hexdump(response)}")
        if response[0] != slave:
            raise ModbusError(f"response from slave {response[0]} instead of {slave}")
        fc = pdu[0]
        if response[1] == (fc | 0x80):
            raise ModbusException(response[2])
        if response[1] != fc:
            raise ModbusError(f"unexpected function code in response: {hexdump(response)}")
        return body[1:]

    def read_registers(self, kind: str, address: int, count: int = 1,
                       slave: Optional[int] = None, timeout: Optional[float] = None) -> List[int]:
        """Reads count registers starting at address: kind "hr" uses FC 03, any other value FC 04.

        slave and timeout, when given, override the client's for this request.
        Returns the values as unsigned 16-bit integers. Raises ValueError for count
        outside 1-125 or addresses past 0xFFFF, ModbusError if the byte count is wrong.
        """
        if not 1 <= count <= 125:
            raise ValueError("you can read 1 to 125 registers at a time")
        if not 0 <= address <= 0xFFFF or address + count - 1 > 0xFFFF:
            raise ValueError("address out of range (0-65535)")
        fc = 3 if kind == "hr" else 4
        s = self.slave if slave is None else slave
        body = self._transact(struct.pack(">BHH", fc, address, count), s, timeout)
        if len(body) < 2 or body[1] != 2 * count or len(body) != 2 + 2 * count:
            raise ModbusError(f"unexpected data length: {hexdump(body)}")
        return list(struct.unpack(f">{count}H", body[2:]))

    def write_register(self, address: int, value: int) -> None:
        """Writes value to holding register address with FC 06 and checks the echo.

        No safety checks: call it only through HeltyShell.safe_write.
        Raises ValueError for an address or value outside 0-65535, ModbusError if the
        echo does not match the request.
        """
        if not 0 <= address <= 0xFFFF:
            raise ValueError("address out of range (0-65535)")
        if not 0 <= value <= 0xFFFF:
            raise ValueError("value out of range (0-65535)")
        pdu = struct.pack(">BHH", 6, address, value)
        body = self._transact(pdu, self.slave)
        if body != pdu:                       # the response to FC 06 echoes the request
            raise ModbusError(f"unexpected write acknowledgement: {hexdump(body)}")


# -----------------------------------------------------------------------------
# Shell
# -----------------------------------------------------------------------------


def find_ports() -> List[str]:
    """Lists the devices matching PORT_PATTERNS, without duplicates and in pattern order."""
    found: List[str] = []
    for pattern in PORT_PATTERNS:
        for p in sorted(glob.glob(pattern)):
            if p not in found:
                found.append(p)
    return found


def guarded(fn):
    """Decorator for the do_* commands: prints the expected errors instead of raising them.

    Sets self.last_ok, used as the exit code in single-command mode: False at the
    start, True only if the command finishes without an exception. For ValueError and
    IndexError it shows the first docstring line as usage; for OSError it closes the
    port, which need_client reopens on the next command. KeyboardInterrupt is not
    caught here: cmdloop handles it.
    """

    def wrapper(self, arg):
        self.last_ok = False
        try:
            result = fn(self, arg)
            self.last_ok = True
            return result
        except WriteRefused as e:
            self.err(f"Write blocked: {e}")
        except Cancelled as e:
            self.hint(str(e))
        except ModbusTimeout as e:
            self.err(f"Timeout: {e}.")
            self.hint("Check the unit's power, A/B polarity and slave address "
                      f"(now {self.slave}). 'debug on' shows the frames, 'identify' "
                      "searches for the unit again.")
        except ModbusError as e:
            self.err(str(e))
        except (ValueError, IndexError) as e:
            self.err(str(e) or "invalid arguments")
            usage = (fn.__doc__ or "").strip().splitlines()
            if usage:
                self.hint("Usage: " + usage[0])
        except OSError as e:
            self.err(f"Serial port error: {e}")
            self.close_port()
        return None

    # cmd.Cmd reads __doc__ for 'help <command>'
    wrapper.__doc__ = fn.__doc__
    wrapper.__name__ = fn.__name__
    return wrapper


class HeltyShell(cmd.Cmd):
    """Interactive shell for the unit, based on cmd.Cmd.

    Each command is a do_<name> method: its docstring is the help shown by
    'help <name>' and its first line is the syntax. Every write goes through safe_write.
    """

    # headers for cmd.Cmd's standard help
    doc_header = "Commands:"
    misc_header = "Topics:"
    undoc_header = "Other commands:"
    ruler = "-"

    # content of 'help' without arguments: (section, [(syntax, description)])
    HELP_SUMMARY = [
        ("Control", [
            ("status", "mode, temperatures and alarms"),
            ("off", "turns the unit off"),
            ("speed <1-4>", "sets the fan speed"),
            ("hyper | night | freecooling", "starts a preset"),
            ("mode [0-7 | name]", "shows or sets the mode"),
            ("monitor [seconds]", "continuous updates, Ctrl+C to stop"),
        ]),
        ("Readings", [
            ("temp", "indoor and outdoor temperature"),
            ("alarms", "alarm register bit by bit"),
            ("info", "connection, firmware and model"),
            ("dump [ir|hr] [from] [to]", "lists registers (default IR 1000-1020)"),
            ("read <ir|hr> <address> [count]", "reads raw registers"),
            ("write <address> <value>", "writes a holding register (see 'safety')"),
        ]),
        ("Unit and bus", [
            ("identify", "finds and checks the unit on the bus"),
            ("scan [from] [to]", "lists every slave that responds (default 1-247)"),
            ("id [n]", "shows or changes the slave address"),
            ("port [path]", "shows or changes the serial port"),
            ("timeout [seconds]", "shows or changes the response timeout"),
            ("debug [on|off]", "shows the Modbus frames sent and received"),
        ]),
        ("Safety", [
            ("safety", "summary of the active safeguards"),
            ("readonly [on|off]", "blocks every write"),
            ("log [n]", "latest writes (default 10)"),
            ("exit", "closes the shell (also Ctrl+D)"),
        ]),
    ]

    def __init__(self, args: argparse.Namespace):
        """Initialises the session from the parse_args options; does not open the port."""
        super().__init__()
        self.port_path: Optional[str] = args.port
        self.baud: int = args.baud
        self.parity: str = args.parity.upper()
        self.id_explicit: bool = args.id is not None    # address chosen by the user: identify looks nowhere else
        self.slave: int = args.id if args.id is not None else DEFAULT_SLAVE
        self.timeout: float = args.timeout
        self.debug: bool = args.debug
        self.read_only: bool = args.read_only
        self.expert: bool = args.expert
        self.ignore_pid: bool = args.ignore_pid
        self.port: Optional[SerialPort] = None
        self.client: Optional[ModbusClient] = None
        self.verified: Dict[Tuple[str, int], Optional[int]] = {}   # (port, slave) -> PID; None = not exposed
        self.last_write = 0.0      # time.time() of the last write in this session
        self.last_ok = True        # outcome of the last command (see guarded)
        self.started = False       # preloop already ran: cmdloop restarts after each Ctrl+C
        self.running = False       # True between precmd and postcmd
        self._update_prompt()

    # -- output --------------------------------------------------------------
    def out(self, s: str = "") -> None:
        """Prints s to stdout."""
        print(s, flush=True)

    def err(self, s: str) -> None:
        """Prints s in red."""
        print(red(s), flush=True)

    def warn(self, s: str) -> None:
        """Prints s in yellow."""
        print(yellow(s), flush=True)

    def hint(self, s: str) -> None:
        """Prints s dimmed."""
        print(dim(s), flush=True)

    def row(self, label: str, value: str) -> None:
        """Prints a "label value" row with the label padded to 20 columns."""
        print(f"  {label:<20}{value}", flush=True)

    def _update_prompt(self) -> None:
        """Updates the prompt with the slave address and the active modes, for example "helty[2 EXPERT]> "."""
        flags = ""
        if self.read_only:
            flags += " read-only"
        if self.expert:
            flags += " EXPERT"
        self.prompt = f"helty[{self.slave}{flags}]> "

    # -- connection ----------------------------------------------------------
    def close_port(self) -> None:
        """Closes the port, if open, and drops the Modbus client."""
        if self.port:
            self.port.close()
        self.port, self.client = None, None

    def open_port(self, quiet: bool = False) -> bool:
        """Opens the serial port and creates the Modbus client. Returns True on success.

        Without port_path it picks the only port found or, when there are several, the
        one where the unit was found last time (STATE_FILE). After opening it listens
        to the bus for BUS_LISTEN_OPEN seconds and warns about any traffic. With quiet
        it prints nothing.
        """
        self.close_port()
        path = self.port_path
        if not path:
            ports = find_ports()
            if not ports:
                if not quiet:
                    self.err("No USB-RS485 adapter found.")
                    self.hint("Connect it, or set the port with 'port <path>'.")
                return False
            if len(ports) == 1:
                path = ports[0]
            else:
                # no attempts on the other ports: opening them can reset boards such as an ESP32
                remembered = load_state().get("port")
                if remembered not in ports:
                    if not quiet:
                        self.err("Several serial ports found, choose one with 'port <path>' "
                                 "or the --port option (the shell then remembers it):")
                        for p in ports:
                            self.out(f"  {p}")
                    return False
                path = remembered
                if not quiet:
                    self.hint(f"Several serial ports connected: using {path}, where the unit "
                              "was found last time.")
        try:
            self.port = SerialPort(path, self.baud, self.parity)
        except OSError as e:
            if not quiet:
                self.err(f"Cannot open {path}: {e}")
                if e.errno == errno.EBUSY:
                    self.hint("The port is already open in another program, for example "
                              "another HeltyShell or a serial monitor.")
            return False
        self.port_path = path
        self.client = ModbusClient(self.port, self.slave, self.timeout,
                                   log=self.out if self.debug else None)
        noise = self.port.listen(BUS_LISTEN_OPEN)
        if noise and not quiet:
            self.warn(f"Unrequested traffic on the bus ({len(noise)} bytes): another master "
                      "is probably connected (for example an ESPHome device).")
            self.hint("Reads may fail: disconnect it before writing, the shell cannot always "
                      "detect it.")
        return True

    def need_client(self) -> ModbusClient:
        """Returns the Modbus client, opening the port if needed. Raises ModbusError if it cannot."""
        if self.client is None and not self.open_port():
            raise ModbusError("serial port not available")
        assert self.client is not None
        return self.client

    def set_slave(self, s: int) -> None:
        """Sets the slave address of shell and client and updates the prompt; does not check the device."""
        self.slave = s
        if self.client:
            self.client.slave = s
        self._update_prompt()

    # -- identification ------------------------------------------------------
    def read_pid(self, s: int, timeout: Optional[float] = None,
                 retry_timeout: bool = True) -> Optional[int]:
        """Reads the PID of slave s from IR 1013.

        Returns None if the slave replies with a Modbus exception, that is, it does not
        expose the register. An invalid frame is requested again up to PID_ATTEMPTS
        times, and so is a missing response when retry_timeout is true. When the
        attempts run out it raises the last ModbusTimeout or ModbusError.
        """
        c = self.need_client()
        for attempt in range(1, PID_ATTEMPTS + 1):
            try:
                return c.read_registers("ir", IR_PID, 1, slave=s, timeout=timeout)[0]
            except ModbusException:
                return None
            except ModbusTimeout:
                if not retry_timeout or attempt == PID_ATTEMPTS:
                    raise
            except ModbusError:
                if attempt == PID_ATTEMPTS:
                    raise
        return None

    def probe(self, s: int, timeout: Optional[float] = None,
              retry_timeout: bool = True) -> Tuple[bool, Optional[int]]:
        """Checks whether slave s responds. Returns (responds, PID or None).

        Invalid frames even after the read_pid attempts count as a response, with PID None.
        """
        try:
            return True, self.read_pid(s, timeout, retry_timeout)
        except ModbusTimeout:
            return False, None
        except ModbusError:
            return True, None

    def scan_bus(self, start: int, end: int) -> List[Tuple[int, Optional[int]]]:
        """Polls the addresses from start to end inclusive; returns [(address, PID or None)] of those that respond.

        Uses SCAN_TIMEOUT and does not retry missing responses. Ctrl+C stops the scan
        and returns the results collected so far.
        """
        found: List[Tuple[int, Optional[int]]] = []
        tty = sys.stdout.isatty()
        try:
            for a in range(start, end + 1):
                if tty:
                    sys.stdout.write(f"\r  trying address {a}/{end}…   ")
                    sys.stdout.flush()
                ok, pid = self.probe(a, SCAN_TIMEOUT, retry_timeout=False)
                if ok:
                    found.append((a, pid))
                    if tty:
                        sys.stdout.write("\r" + " " * 40 + "\r")
                    model = KNOWN_PIDS.get(pid, "unknown model") if pid is not None \
                        else "does not expose the PID"
                    self.out(green(f"  ✓ slave {a}") + dim(f"  {model}"))
        except KeyboardInterrupt:
            self.out("")
            self.hint("Scan interrupted.")
        if tty:
            sys.stdout.write("\r" + " " * 40 + "\r")
            sys.stdout.flush()
        return found

    def remember_slave(self, s: int) -> None:
        """Saves to STATE_FILE the address s for the current port, and the port as the last one used with the unit."""
        if not self.port_path:
            return
        state = load_state()
        slaves = state.get("slaves") if isinstance(state.get("slaves"), dict) else {}
        slaves[self.port_path] = s
        state["slaves"] = slaves
        state["port"] = self.port_path
        save_state(state)

    def adopt(self, s: int, pid: Optional[int]) -> None:
        """Makes s the current slave and prints the outcome of the identification.

        With a known PID it also remembers port and address (remember_slave).
        """
        self.set_slave(s)
        if self.port_path and pid is not None:
            # a missing PID is not cached: it may come from noise on the bus,
            # and verify_identity reads it again before writing
            self.verified[(self.port_path, s)] = pid
        if pid in KNOWN_PIDS:
            self.remember_slave(s)
            self.out(green("✓ ") + f"Unit found: slave {s}, {KNOWN_PIDS[pid]} (PID 0x{pid:04X})")
        elif pid is None:
            self.warn(f"Slave {s} responds but its PID (IR 1013) is unreadable: "
                      "the shell will check it again before writing.")
        else:
            blocked = "PID check disabled by --ignore-pid" if self.ignore_pid \
                else "writes blocked"
            self.warn(f"Slave {s} responds but is not a known unit (PID 0x{pid:04X}): {blocked}.")

    def identify(self) -> bool:
        """Looks for the unit and makes it the current slave. Returns True if found.

        Tries the explicit address, or the one remembered for the port and then
        DEFAULT_SLAVE: the first that responds is adopted, and the result is True if it
        has a known PID or --ignore-pid is on. If none responds and the address is not
        explicit, it scans the whole bus and adopts the unit only if it finds exactly one.
        """
        self.need_client()
        if self.id_explicit:
            candidates = [self.slave]
        else:
            remembered = load_state().get("slaves", {}).get(self.port_path or "")
            candidates = []
            for s in (remembered, DEFAULT_SLAVE):
                if isinstance(s, int) and 1 <= s <= 247 and s not in candidates:
                    candidates.append(s)
        for s in candidates:
            ok, pid = self.probe(s)
            if ok:
                self.adopt(s, pid)
                return pid in KNOWN_PIDS or self.ignore_pid
        if self.id_explicit:
            self.err(f"No response from the unit at address {self.slave}.")
            self.hint("Use 'identify' to search the whole bus.")
            return False
        self.out(f"The unit does not respond at the known addresses ({', '.join(map(str, candidates))}): "
                 "searching the bus, Ctrl+C to stop.")
        found = self.scan_bus(1, 247)
        helty = [(s, p) for s, p in found if p in KNOWN_PIDS]
        if len(helty) == 1:
            self.adopt(*helty[0])
            if len(found) > 1:
                self.hint("Other devices are on the bus too: writes go only to the selected unit.")
            return True
        if not found:
            self.err("No device responded.")
            self.hint("Check power, A/B polarity and that no other master is connected.")
            return False
        if len(helty) > 1:
            self.warn("Several units on the bus: choose one with 'id <n>'.")
        else:
            self.warn("Devices respond, but none is a known unit.")
        return False

    def verify_identity(self) -> int:
        """Before a write, checks that the current slave is a known unit and returns its PID.

        The PID is read once per (port, slave), then taken from the cache. Raises
        ModbusTimeout if the slave does not respond, WriteRefused if the PID is
        unreadable, not exposed or not in KNOWN_PIDS (the last case is allowed with
        --ignore-pid).
        """
        key = (self.port_path or "", self.slave)
        if key not in self.verified:
            try:
                pid = self.read_pid(self.slave)
            except ModbusTimeout:
                raise ModbusTimeout("no response from the unit while checking its model")
            except ModbusError as e:
                raise WriteRefused(f"the unit's PID is unreadable ({e}). Try again.")
            self.verified[key] = pid
        pid = self.verified[key]
        if pid is None:
            raise WriteRefused(f"slave {self.slave} does not expose the PID (IR 1013): "
                               "it is not a Helty unit.")
        if pid not in KNOWN_PIDS and not self.ignore_pid:
            raise WriteRefused(f"unknown PID 0x{pid:04X} (expected 0x1102, Flow40). "
                               "If you are sure about the device, start with --ignore-pid.")
        return pid

    # -- protected write -----------------------------------------------------
    def log_write(self, address: int, before: Optional[int], requested: int,
                  after: Optional[int], outcome: str) -> None:
        """Appends to WRITE_LOG a line with date, port, slave, register, previous, requested and read-back values, outcome.

        If the file is not writable it prints a warning; never raises.
        """
        stamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        line = (f"{stamp}  port={self.port_path}  slave={self.slave}  HR {address}: "
                f"{before} -> {requested}  readback={after}  result={outcome}\n")
        try:
            with open(WRITE_LOG, "a", encoding="utf-8") as f:
                f.write(line)
        except OSError:
            self.warn(f"Cannot update the log {WRITE_LOG}")

    def safe_write(self, address: int, value: int) -> Tuple[Optional[int], bool]:
        """Writes value to holding register address, applying every safeguard in order.

        This is the only path for writes. Returns (read-back value, True) after a write,
        or (current value, False) if the register already held value and nothing was
        sent. Raises WriteRefused if a safeguard blocks, Cancelled if the user cancels
        the confirmation, ModbusError if the unit does not acknowledge the write or
        cannot be read back.
        """
        # 1. read-only mode
        if self.read_only:
            raise WriteRefused("read-only mode is on ('readonly off' to turn it off).")
        # 2. protected registers, never written
        if address in PROTECTED_HR:
            raise WriteRefused(f"HR {address} is protected (password and non-volatile data): "
                               "the shell never writes it.")
        # 3. allowed register and value: WRITE_RULES, or any register with --expert
        rule = WRITE_RULES.get(address)
        if rule is None and not self.expert:
            raise WriteRefused(f"HR {address} is not writable: only HR 1000 (machine state) "
                               "is allowed. Start with --expert for other registers.")
        lo, hi = rule if rule else (0, 0xFFFF)
        if not lo <= value <= hi:
            raise WriteRefused(f"value {value} is outside the allowed range for HR {address} "
                               f"({lo}-{hi}).")
        c = self.need_client()
        # 4. identity of the unit
        self.verify_identity()
        # 5. current value: if it matches, nothing is sent
        current = c.read_registers("hr", address, 1)[0]
        if current == value:
            return current, False
        # 6. minimum interval between writes; turning the unit off is always allowed
        is_shutdown = address == HR_STATUS and value == 0
        state = load_state()
        last = state.get("last_write", 0)
        if not isinstance(last, (int, float)):
            last = 0
        # STATE_FILE covers separate runs, self.last_write the case where the file is not updated
        elapsed = time.time() - max(last, self.last_write)
        if not is_shutdown and 0 <= elapsed < WRITE_COOLDOWN:
            raise WriteRefused(f"write too soon after the previous one: try again in "
                               f"{fmt_s(round(WRITE_COOLDOWN - elapsed, 1))}.")
        # 7. typed confirmation for registers outside WRITE_RULES (expert mode only)
        if rule is None:
            self.warn(f"EXPERT MODE: HR {address} from {current} to {value}. "
                      "An unknown register can change the unit's configuration.")
            if ask(f"To confirm, type the register number ({address}): ") != str(address):
                raise Cancelled("Write cancelled.")
        # 8. free bus: blocks if it hears traffic (best effort, not guaranteed)
        assert self.port is not None
        noise = self.port.listen(BUS_LISTEN_WRITE)
        if noise:
            raise WriteRefused(f"traffic detected on the bus ({len(noise)} bytes): another master "
                               "is probably connected (for example an ESPHome device). "
                               "Disconnect it and try again.")
        # 9. write without automatic retries; from here on Ctrl+C is ignored
        with uninterruptible():
            now = time.time()
            self.last_write = now
            state["last_write"] = now
            if not save_state(state):
                self.warn(f"Cannot update {STATE_FILE}: the minimum interval between "
                          "writes applies to this session only.")
            try:
                c.write_register(address, value)
            except ModbusTimeout:
                # without an echo there is no telling whether the unit applied the value:
                # it is read back and logged, and the write is not repeated
                time.sleep(0.2)
                try:
                    back: Optional[int] = c.read_registers("hr", address, 1)[0]
                except ModbusError:
                    back = None
                self.log_write(address, current, value, back, "unconfirmed")
                back_txt = "unreadable" if back is None else str(back)
                raise ModbusError(f"the unit did not acknowledge the write and the shell will NOT "
                                  f"repeat it. Current value of HR {address}: {back_txt}.")
            except ModbusError as e:
                self.log_write(address, current, value, None, f"error: {e}")
                raise
            # 10. read-back and log
            time.sleep(0.05)
            try:
                back = c.read_registers("hr", address, 1)[0]
            except ModbusError:
                self.log_write(address, current, value, None, "written, read-back failed")
                raise
            self.log_write(address, current, value, back, "ok" if back == value else "different value")
            return back, True

    def confirm_faults_before_start(self) -> bool:
        """Returns True if the unit may be started: no FAULT_MASK fault active, or the user confirms."""
        alarms = self.need_client().read_registers("ir", IR_ALARMS, 1)[0]
        faults = [name for bit, name, _ in ALARM_BITS if bit & FAULT_MASK and alarms & bit]
        if not faults:
            return True
        self.warn("The unit reports: " + ", ".join(faults) + ".")
        self.warn("Starting it with an active fault can make things worse.")
        return ask("Continue anyway? [y/N] ") in ("y", "yes")

    # -- high-level readings -------------------------------------------------
    def read_ir_block(self, start: int, count: int) -> List[Optional[int]]:
        """Reads count input registers from start with a single request.

        If the unit replies with an exception (for example for a register of the block
        it does not implement) they are read again one by one; those that raise an
        exception are None.
        """
        c = self.need_client()
        try:
            return list(c.read_registers("ir", start, count))
        except ModbusException:
            values: List[Optional[int]] = []
            for a in range(start, start + count):
                try:
                    values.append(c.read_registers("ir", a, 1)[0])
                except ModbusException:
                    values.append(None)
            return values

    def read_mode(self) -> int:
        """Returns the current value of HR_STATUS."""
        return self.need_client().read_registers("hr", HR_STATUS, 1)[0]

    def set_mode(self, value: int) -> None:
        """Puts the unit in mode value (0-7) through safe_write and prints the outcome.

        If the mode changes to a value other than 0 and the unit reports a fault, it
        asks for confirmation. Raises ValueError for values outside MODES; WriteRefused
        and Cancelled like safe_write.
        """
        if value not in MODES:
            raise ValueError("invalid mode: use a number from 0 to 7")
        # read-only mode and model are checked before the fault question, so the user is
        # not asked to confirm a write that would be blocked anyway
        if self.read_only:
            raise WriteRefused("read-only mode is on ('readonly off' to turn it off).")
        self.verify_identity()
        if value != 0 and value != self.read_mode() and not self.confirm_faults_before_start():
            raise Cancelled("Command cancelled.")
        back, written = self.safe_write(HR_STATUS, value)
        if not written:
            self.out(green("✓ ") + f"Mode already set: {bold(mode_name(value))}")
        elif back == value:
            self.out(green("✓ ") + f"Mode: {bold(mode_name(back))}")
        else:
            self.warn(f"! Sent {mode_name(value)}, the unit reports {mode_name(back)}")

    def snapshot(self) -> Tuple[int, List[Optional[int]]]:
        """Reads HR_STATUS and the input registers from IR_FIRST to IR_ALARMS. Returns (mode, registers or None)."""
        mode = self.read_mode()
        regs = self.read_ir_block(IR_FIRST, IR_ALARMS - IR_FIRST + 1)
        return mode, regs

    # -- shell loop ----------------------------------------------------------
    def cmdloop(self, intro: Optional[str] = None) -> None:
        """Command loop: Ctrl+C interrupts the running command or the line being typed, not the shell.

        cmd.Cmd.cmdloop ends with KeyboardInterrupt: it is restarted here, and preloop
        does not repeat the start-up thanks to self.started.
        """
        while True:
            try:
                return super().cmdloop(intro)
            except KeyboardInterrupt:
                self.out("")
                self.hint("Command interrupted." if self.running
                          else "To quit type 'exit' or press Ctrl+D.")
                self.running = False
                self._update_prompt()

    def preloop(self) -> None:
        """Start-up of the interactive session, once only: readline, port opening, identification."""
        if self.started:
            return
        self.started = True
        _setup_readline()
        self.out(bold("HeltyShell") + " - Helty Flow40 control over Modbus RTU")
        if self.open_port(quiet=False) and self.port:
            self.out(f"Port: {self.port.describe()}")
            try:
                self.identify()
            except (ModbusError, OSError) as e:
                self.err(f"Identification failed: {e}")
            except KeyboardInterrupt:
                self.out("")
                self.hint("Identification interrupted: run 'identify' to try again.")
        if self.read_only:
            self.hint("Read-only mode: no write will be sent.")
        if self.expert:
            self.warn("Expert mode: registers other than HR 1000 are writable too, "
                      "always with a typed confirmation.")
        self.out(dim("Type 'help' for the list of commands, 'exit' to quit."))
        self._update_prompt()

    def emptyline(self) -> bool:
        """Empty line: does nothing. cmd.Cmd's default repeats the last command, which can be a write."""
        return False

    def default(self, line: str) -> bool:
        """Line without a matching do_*: runs the mode alias (MODE_ALIASES) or prints an error."""
        word = line.split()[0] if line.split() else line
        if word.lower() in MODE_ALIASES:
            return bool(self.onecmd(f"mode {word}"))
        self.err(f"Unknown command: {word}. Type 'help' for the list.")
        self.last_ok = False
        return False

    def precmd(self, line: str) -> str:
        """Marks the start of a command, so cmdloop can tell Ctrl+C at the prompt from Ctrl+C during a command."""
        self.running = True
        return line

    def postcmd(self, stop: bool, line: str) -> bool:
        """Marks the end of the command and updates the prompt, since slave or modes may have changed."""
        self.running = False
        self._update_prompt()
        return stop

    def do_help(self, arg: str) -> None:
        """help [command] - list of commands, or details of one."""
        if arg:
            fn = getattr(self, "do_" + arg, None)
            if fn is not None and fn.__doc__:
                # cleandoc removes the indentation of the lines after the first (cmd.Cmd does it only from Python 3.13)
                self.out(inspect.cleandoc(fn.__doc__))
                return None
            return super().do_help(arg)
        for section, items in self.HELP_SUMMARY:
            self.out(bold(section))
            for name, desc in items:
                self.out(f"  {name:<36}{dim(desc)}")
        self.out(dim("Examples: 'speed 2', 'night', 'monitor 10', 'read ir 1000 2'."))
        return None

    # -- control -------------------------------------------------------------
    @guarded
    def do_status(self, arg: str) -> None:
        """status - mode, temperatures and alarms of the unit.

        Reads HR 1000 and IR 1000-1006. The "sensor missing" bits, expected on the
        Flow40, are not listed as alarms; a warning is shown if they clear.
        """
        mode, regs = self.snapshot()
        t_amb, t_ext, alarms = regs[0], regs[1], regs[IR_ALARMS - IR_FIRST]
        self.row("Mode", bold(f"{mode_name(mode)}") + dim(f"  ({mode})"))
        self.row("Indoor temp.", fmt_temp(t_amb) if t_amb is not None else "n/a")
        self.row("Outdoor temp.", fmt_temp(t_ext) if t_ext is not None else "n/a")
        if alarms is None:
            self.row("Alarms", "n/a")
        else:
            active, _ = decode_alarms(alarms)
            self.row("Alarms", red(", ".join(active)) if active else green("none"))
            if alarms & EXPECTED_FLAGS != EXPECTED_FLAGS:
                self.row("", yellow(f"IR 1006 = 0x{alarms:04X}: 'sensor missing' flags differ from usual"))

    @guarded
    def do_off(self, arg: str) -> None:
        """off - turns the unit off (HR 1000 = 0).

        Allowed even within the minimum interval between two writes.
        """
        self.set_mode(0)

    @guarded
    def do_speed(self, arg: str) -> None:
        """speed <1-4> - sets the fan speed (HR 1000 = 1-4)."""
        v = parse_int(arg.split()[0])
        if not 1 <= v <= 4:
            raise ValueError("speed must be 1 to 4")
        self.set_mode(v)

    @guarded
    def do_hyper(self, arg: str) -> None:
        """hyper - starts the Hyper preset (HR 1000 = 5)."""
        self.set_mode(5)

    @guarded
    def do_night(self, arg: str) -> None:
        """night - starts the Night preset (HR 1000 = 6)."""
        self.set_mode(6)

    @guarded
    def do_freecooling(self, arg: str) -> None:
        """freecooling - starts the Free cooling preset (HR 1000 = 7)."""
        self.set_mode(7)

    @guarded
    def do_mode(self, arg: str) -> None:
        """mode [0-7 | name] - without arguments shows the current mode and the list, otherwise sets it.

        Accepts the number or a name, for example off, v2, boost, night, fc.
        """
        a = arg.strip().lower()
        if not a:
            current = self.read_mode()
            for v, name in MODES.items():
                mark = green("●") if v == current else " "
                self.out(f"  {mark} {v}  {name}")
            return
        if a in MODE_ALIASES:
            value = MODE_ALIASES[a]
        else:
            value = parse_int(a)
        self.set_mode(value)

    def complete_mode(self, text, line, begidx, endidx):
        """Completes mode numbers and names."""
        names = sorted(set(MODE_ALIASES) | {str(v) for v in MODES})
        return [n for n in names if n.startswith(text.lower())]

    @guarded
    def do_monitor(self, arg: str) -> None:
        """monitor [seconds] - reads the status at intervals (default 5 s), Ctrl+C to stop.

        Reads only. A mode that differs from the previous reading is highlighted.
        """
        interval = float(arg.split()[0].replace(",", ".")) if arg.split() else 5.0
        if interval < 1:
            raise ValueError("the minimum interval is 1 second")
        self.out(dim(f"Updating every {fmt_s(interval)}, reads only. Ctrl+C to stop."))
        last_mode: Optional[int] = None
        try:
            while True:
                stamp = datetime.now().strftime("%H:%M:%S")
                try:
                    mode, regs = self.snapshot()
                    t_amb, t_ext, alarms = regs[0], regs[1], regs[IR_ALARMS - IR_FIRST]
                    mode_txt = f"{mode_name(mode):<13}"
                    if last_mode is not None and mode != last_mode:
                        mode_txt = yellow(mode_txt)
                    last_mode = mode
                    amb = fmt_temp(t_amb) if t_amb is not None else "n/a"
                    ext = fmt_temp(t_ext) if t_ext is not None else "n/a"
                    if alarms is None:
                        al = yellow("alarms n/a")
                    else:
                        active, _ = decode_alarms(alarms)
                        al = red(", ".join(active)) if active else dim("no alarms")
                    self.out(f"{dim(stamp)}  {mode_txt}  in {amb:>8}  out {ext:>8}  {al}")
                except ModbusError as e:
                    # a read error is printed and the monitor keeps going
                    self.out(f"{dim(stamp)}  {red(str(e))}")
                time.sleep(interval)
        except KeyboardInterrupt:
            self.out("")
            self.hint("Monitor stopped.")

    # -- readings ------------------------------------------------------------
    @guarded
    def do_temp(self, arg: str) -> None:
        """temp - indoor and outdoor temperature (IR 1000-1001)."""
        t_amb, t_ext = self.need_client().read_registers("ir", IR_T_AMB, 2)
        self.row("Indoor temp.", fmt_temp(t_amb))
        self.row("Outdoor temp.", fmt_temp(t_ext))

    @guarded
    def do_alarms(self, arg: str) -> None:
        """alarms - alarm register (IR 1006) bit by bit."""
        v = self.need_client().read_registers("ir", IR_ALARMS, 1)[0]
        self.out(f"  IR 1006 = {v} (0x{v:04X})")
        for bit, name, real in ALARM_BITS:
            on = bool(v & bit)
            if on:
                mark = red("●") if real else dim("●")
            else:
                mark = dim("○")
            text = name if on and real else dim(name)
            self.out(f"  {mark} 0x{bit:04X}  {text}")

    @guarded
    def do_info(self, arg: str) -> None:
        """info - connection settings, firmware (IR 1012) and model (PID, IR 1013)."""
        self.row("Port", self.port.describe() if self.port else (self.port_path or "not open"))
        self.row("Slave", str(self.slave))
        self.row("Timeout", fmt_s(self.timeout))
        fw, pid = self.read_ir_block(IR_FW, 2)
        self.row("Firmware", str(fw) if fw is not None else "n/a")
        self.row("PID", decode_pid(pid) if pid is not None else "n/a")

    @guarded
    def do_dump(self, arg: str) -> None:
        """dump [ir|hr] [from] [to] - lists registers one by one (default: ir 1000 1020).

        One request per register, 200 registers at most; with only the start it reads 21.
        Registers that reply with an exception are shown as such.
        """
        parts = arg.split()
        kind = "ir"
        if parts and parts[0].lower() in ("ir", "hr"):
            kind = parts.pop(0).lower()
        start = parse_int(parts[0]) if parts else 1000
        end = parse_int(parts[1]) if len(parts) > 1 else (start + 20 if parts else 1020)
        if end < start or end - start > 200:
            raise ValueError("invalid range (200 registers at most)")
        labels = IR_LABELS if kind == "ir" else HR_LABELS
        c = self.need_client()
        self.out(dim(f"  {'reg':>6}  {'dec':>6}  {'hex':>6}  meaning"))
        for a in range(start, end + 1):
            try:
                v = c.read_registers(kind, a, 1)[0]
                self.out(f"  {a:>6}  {v:>6}  0x{v:04X}  {dim(labels.get(a, ''))}")
            except ModbusException as e:
                self.out(f"  {a:>6}  {dim('-'):>6}  {dim('exception ' + hex(e.code))}")

    @guarded
    def do_read(self, arg: str) -> None:
        """read <ir|hr> <address> [count] - reads raw registers (input or holding).

        A single request for 1 to 125 registers; each value in decimal, hex and signed.
        """
        parts = arg.split()
        kind = parts[0].lower()
        if kind not in ("ir", "hr"):
            raise ValueError("specify 'ir' (input) or 'hr' (holding)")
        address = parse_int(parts[1])
        count = parse_int(parts[2]) if len(parts) > 2 else 1
        values = self.need_client().read_registers(kind, address, count)
        labels = IR_LABELS if kind == "ir" else HR_LABELS
        for i, v in enumerate(values):
            a = address + i
            self.out(f"  {kind.upper()} {a:>5} = {v:>6}  0x{v:04X}  {signed16(v):>6}  "
                     f"{dim(labels.get(a, ''))}")

    def complete_read(self, text, line, begidx, endidx):
        """Completes the register type (ir, hr); also used by dump."""
        return [k for k in ("ir", "hr") if k.startswith(text.lower())]

    complete_dump = complete_read

    @guarded
    def do_write(self, arg: str) -> None:
        """write <address> <value> - writes a holding register: only HR 1000 (0-7), others with --expert.

        On HR 1000 with a value 0-7 it behaves like 'mode', fault confirmation included.
        The list of safeguards is in the 'safety' command.
        """
        parts = arg.split()
        address, value = parse_int(parts[0]), parse_int(parts[1])
        if address == HR_STATUS:
            # outside 0-7 safe_write refuses the value and states the allowed range
            self.set_mode(value) if value in MODES else self.safe_write(address, value)
            return
        back, written = self.safe_write(address, value)
        if not written:
            self.out(green("✓ ") + f"HR {address} is already {value}: nothing written.")
            return
        mark = green("✓") if back == value else yellow("!")
        self.out(f"{mark} HR {address} = {back}" + ("" if back == value else f" (sent {value})"))

    # -- unit and bus --------------------------------------------------------
    @guarded
    def do_identify(self, arg: str) -> None:
        """identify - finds the unit (remembered address, then the whole bus) and checks its model.

        Clears the cache of verified PIDs and is not limited to an address set with --id or 'id'.
        """
        self.id_explicit = False
        self.verified.clear()
        if not self.identify():
            raise ModbusError("unit not identified")

    @guarded
    def do_scan(self, arg: str) -> None:
        """scan [from] [to] - lists the slaves that respond and their model (default 1-247).

        Does not change the current slave. The whole bus takes about 40 s.
        """
        parts = arg.split()
        start = parse_int(parts[0]) if parts else 1
        end = parse_int(parts[1]) if len(parts) > 1 else 247
        if not (1 <= start <= end <= 247):
            raise ValueError("invalid address range (1-247)")
        found = self.scan_bus(start, end)
        if found:
            self.out(f"Found: {', '.join(str(s) for s, _ in found)}")
            if all(s != self.slave for s, _ in found) and len(found) == 1:
                self.hint(f"To use it: id {found[0][0]}")
        else:
            self.warn("No slave responded.")

    @guarded
    def do_id(self, arg: str) -> None:
        """id [n] - shows or changes the slave address of the unit (1-247) and checks its model.

        If address n does not respond it is set anyway, with a warning.
        """
        if not arg.strip():
            pid = self.verified.get((self.port_path or "", self.slave))
            model = KNOWN_PIDS.get(pid, "not verified") if pid is not None else "not verified"
            self.out(f"Slave: {self.slave} ({model})")
            return
        v = parse_int(arg.split()[0])
        if not 1 <= v <= 247:
            raise ValueError("the address must be 1 to 247")
        self.id_explicit = True
        ok, pid = self.probe(v)
        if not ok:
            self.set_slave(v)
            self.warn(f"Slave set to {v}, but it does not respond.")
            return
        self.adopt(v, pid)

    @guarded
    def do_port(self, arg: str) -> None:
        """port [path] - shows the current and available ports, or opens another one.

        Opening a port clears the cache of verified PIDs and identifies the unit again.
        """
        if not arg.strip():
            self.out(f"In use: {self.port.describe() if self.port else 'none'}")
            ports = find_ports()
            self.out("Available: " + (", ".join(ports) if ports else "none"))
            return
        self.port_path = arg.strip()
        self.verified.clear()
        if self.open_port() and self.port:
            self.out(green("✓ ") + f"Port open: {self.port.describe()}")
            self.identify()

    def complete_port(self, text, line, begidx, endidx):
        """Completes the port path among those returned by find_ports.

        readline splits the word on '/' and '-', so text is only the last piece: the
        whole path typed so far is compared, and the part replacing text is returned.
        """
        parts = line[:endidx].split(None, 1)
        typed = parts[1] if len(parts) > 1 else ""
        cut = len(typed) - len(text)
        return [p[cut:] for p in find_ports() if p.startswith(typed)]

    def do_timeout(self, arg: str) -> None:
        """timeout [seconds] - shows or changes the maximum wait for a response (0.05-10 s)."""
        if not arg.strip():
            self.out(f"Timeout: {fmt_s(self.timeout)}")
            return
        try:
            v = float(arg.split()[0].replace(",", "."))
        except ValueError:
            self.err("invalid value")
            return
        if not 0.05 <= v <= 10:
            self.err("the timeout must be 0.05 to 10 seconds")
            return
        self.timeout = v
        if self.client:
            self.client.timeout = v
        self.out(f"Timeout set to {fmt_s(v)}.")

    def do_debug(self, arg: str) -> None:
        """debug [on|off] - shows the Modbus frames sent and received; without an argument it toggles."""
        a = arg.strip().lower()
        if a in ("on", "1", "yes"):
            self.debug = True
        elif a in ("off", "0", "no"):
            self.debug = False
        else:
            self.debug = not self.debug
        if self.client:
            self.client.log = self.out if self.debug else None
        self.out(f"Debug {'on' if self.debug else 'off'}.")

    def complete_debug(self, text, line, begidx, endidx):
        """Completes on and off; also used by readonly."""
        return [k for k in ("on", "off") if k.startswith(text.lower())]

    # -- safety --------------------------------------------------------------
    def do_safety(self, arg: str) -> None:
        """safety - summary of the write safeguards and of their current state."""
        pid = self.verified.get((self.port_path or "", self.slave))
        if pid is None:
            ident = "not verified yet"
        else:
            ident = f"PID 0x{pid:04X} " + (green(KNOWN_PIDS[pid]) if pid in KNOWN_PIDS
                                           else red("unknown"))
        state = load_state()
        last = state.get("last_write")
        last_txt = datetime.fromtimestamp(last).strftime("%Y-%m-%d %H:%M:%S") \
            if isinstance(last, (int, float)) else "never"
        self.row("Read-only", yellow("yes") if self.read_only else "no")
        self.row("Expert mode", yellow("yes") if self.expert else "no")
        self.row("Writable", "HR 1000 (machine state), values 0-7" +
                 (" + other registers with confirmation" if self.expert else ""))
        self.row("Never written", "HR 0-15 (password and non-volatile data)")
        self.row("Model", ident + (yellow("  (PID check disabled)") if self.ignore_pid else ""))
        self.row("Same value", "no write sent")
        self.row("Minimum interval", f"{fmt_s(WRITE_COOLDOWN)} between two writes (except turning off)")
        self.row("Bus", f"{fmt_s(BUS_LISTEN_WRITE)} of listening before writing: best effort, "
                 "disconnect other masters")
        self.row("Active faults", "starting the unit requires confirmation")
        self.row("Unacknowledged", "no automatic retry, value read back")
        self.row("Broadcast", "disabled")
        self.row("Serial port", "opened exclusively")
        self.row("Last write", last_txt)
        self.row("Log", WRITE_LOG)

    def do_readonly(self, arg: str) -> None:
        """readonly [on|off] - blocks or allows every write; without an argument it toggles."""
        a = arg.strip().lower()
        if a in ("on", "1", "yes"):
            self.read_only = True
        elif a in ("off", "0", "no"):
            self.read_only = False
        else:
            self.read_only = not self.read_only
        self.out(f"Read-only mode {'on' if self.read_only else 'off'}.")

    complete_readonly = complete_debug

    def do_log(self, arg: str) -> None:
        """log [n] - shows the last n logged writes (default 10)."""
        try:
            n = int(arg.split()[0]) if arg.split() else 10
        except ValueError:
            self.err("invalid number")
            return
        try:
            with open(WRITE_LOG, "r", encoding="utf-8") as f:
                lines = f.readlines()
        except OSError:
            self.out("No writes logged.")
            return
        for line in lines[-n:]:
            self.out("  " + line.rstrip())

    def do_exit(self, arg: str) -> bool:
        """exit - closes the port and the shell."""
        self.close_port()
        return True

    do_quit = do_exit

    def do_EOF(self, arg: str) -> bool:
        """Ctrl+D: closes the shell like 'exit'."""
        self.out("")
        return self.do_exit(arg)


# -----------------------------------------------------------------------------
# Start-up
# -----------------------------------------------------------------------------

# Commands that run in single-command mode without identifying the unit first:
# they do not use the bus, or they look for the slave themselves
NO_IDENTIFY = {"help", "scan", "port", "id", "identify", "safety", "log",
               "timeout", "debug", "readonly"}


def _setup_readline() -> None:
    """Configures readline: Tab completion and history in HISTORY_FILE (500 lines, saved on exit).

    Does nothing without the readline module.
    """
    try:
        import readline
    except ImportError:
        return
    doc = getattr(readline, "__doc__", "") or ""
    if "libedit" in doc:                      # libedit (system Python on macOS) has a different syntax
        readline.parse_and_bind("bind ^I rl_complete")
    else:
        readline.parse_and_bind("tab: complete")
    try:
        readline.read_history_file(HISTORY_FILE)
    except (OSError, IOError):
        pass
    readline.set_history_length(500)

    def _save() -> None:
        """Writes the history to HISTORY_FILE; registered with atexit, ignores errors."""
        try:
            readline.write_history_file(HISTORY_FILE)
        except (OSError, IOError):
            pass

    atexit.register(_save)


def parse_args(argv: List[str]) -> argparse.Namespace:
    """Parses the command line; the arguments after the options end up in args.command."""
    p = argparse.ArgumentParser(
        prog="helty",
        description="HeltyShell - interactive shell for a Helty Flow40 ventilation unit over Modbus RTU.",
        epilog="Examples: 'helty' opens the shell; 'helty status' or 'helty speed 2' run one command and exit.",
    )
    p.add_argument("--port", "-p", help="serial port (default: detected automatically)")
    p.add_argument("--id", "-a", type=int, default=None,
                   help="slave address of the unit (default: found automatically)")
    p.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD,
                   help=f"baud rate (default {DEFAULT_BAUD})")
    p.add_argument("--parity", "-P", default=DEFAULT_PARITY, choices=["N", "E", "O", "n", "e", "o"],
                   help="parity (default N)")
    p.add_argument("--timeout", "-t", type=float, default=DEFAULT_TIMEOUT,
                   help=f"response timeout in seconds (default {DEFAULT_TIMEOUT})")
    p.add_argument("--debug", "-d", action="store_true", help="shows the Modbus frames")
    p.add_argument("--read-only", "-r", action="store_true", help="blocks every write")
    p.add_argument("--expert", action="store_true",
                   help="allows writing registers other than HR 1000, with a typed confirmation")
    p.add_argument("--ignore-pid", action="store_true",
                   help="allows writes even if the model (PID) is not recognised")
    p.add_argument("command", nargs=argparse.REMAINDER, help="command to run without opening the shell")
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    """Entry point; returns the exit code (2 if --id is outside 1-247).

    With a command: opens the port, identifies the unit if the command needs it (see
    NO_IDENTIFY), runs it and returns 0 on success, 1 otherwise, 130 on Ctrl+C.
    Without a command it starts the interactive shell and returns 0.
    """
    args = parse_args(sys.argv[1:] if argv is None else argv)
    if args.id is not None and not 1 <= args.id <= 247:
        print(red("The slave address must be 1 to 247."))
        return 2
    shell = HeltyShell(args)
    if args.command:
        try:
            if not shell.open_port():
                return 1
            first = args.command[0].lower()
            if first not in NO_IDENTIFY:
                try:
                    identified = shell.identify()
                except (ModbusError, OSError) as e:
                    print(red(f"Identification failed: {e}"))
                    identified = False
                if not identified:
                    return 1
            shell.onecmd(" ".join(args.command))
            return 0 if shell.last_ok else 1
        except KeyboardInterrupt:
            print()
            return 130
        finally:
            shell.close_port()
    try:
        shell.cmdloop()
    except KeyboardInterrupt:
        print()
    finally:
        shell.close_port()
    return 0


if __name__ == "__main__":
    sys.exit(main())
