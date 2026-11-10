#!/usr/bin/env python3
"""Tests for HeltyShell against a Flow40 simulated on a pseudo-terminal (pty).

Neither the unit nor the adapter is needed: the shell talks to a simulated Modbus
slave that answers like the Flow40 and, on request, corrupts its responses. State
and log files go to a temporary directory, the real ones are never touched.

    python3 -m unittest -v test_helty_shell
"""

import contextlib
import errno
import io
import os
import select
import signal
import struct
import tempfile
import threading
import unittest
from unittest import mock

import helty_shell as hs

hs._COLOR = False    # output without ANSI sequences, to compare it as text


class Flow40:
    """Simulated Modbus RTU slave that answers like a Flow40, served by a thread on a pty.

    Handles FC 03, 04 and 06; a register missing from hr or ir produces exception 2.
    """

    def __init__(self, slave: int = 2):
        """Creates the pty, sets the registers of a Flow40 that is off and has no alarms, starts the thread."""
        self.slave = slave
        self.hr = {1000: 0}
        self.ir = {a: 0 for a in range(1000, 1021)}
        self.ir.update({1000: 215, 1001: 0xFFE2, 1004: 400, 1006: 0x5000, 1012: 117, 1013: 0x1102})
        self.garble = 0          # number of responses to send with a wrong CRC
        self.noise = 0           # number of responses to prefix with a stray 0x00 byte
        self.writes = []         # (register, value) of every accepted FC 06, in order
        self.on_write = None     # called when an FC 06 arrives, before the response
        # the slave side stays open here too, so reads from the master do not fail with EIO
        # when the shell closes and reopens the port
        self.master, self._slave_fd = os.openpty()
        self.path = os.ttyname(self._slave_fd)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._serve, daemon=True)
        self._thread.start()

    def close(self) -> None:
        """Stops the thread and closes both sides of the pty."""
        self._stop.set()
        self._thread.join()
        os.close(self.master)
        os.close(self._slave_fd)

    def _serve(self) -> None:
        """Reads requests from the master side and answers the valid ones addressed to self.slave.

        On a wrong CRC it drops one byte and tries again, to resynchronise on the frame.
        """
        buf = bytearray()
        while not self._stop.is_set():
            if not select.select([self.master], [], [], 0.05)[0]:
                continue
            buf += os.read(self.master, 256)
            while len(buf) >= 8:             # FC 03, 04 and 06 requests are 8 bytes long
                frame = bytes(buf[:8])
                if hs.crc16(frame[:6]) != frame[6] | (frame[7] << 8):
                    del buf[0]
                    continue
                del buf[:8]
                if frame[0] == self.slave:
                    os.write(self.master, self._answer(frame))

    def _answer(self, frame: bytes) -> bytes:
        """Builds the response to a valid request and applies the requested disturbances (garble, noise)."""
        fc = frame[1]
        addr, val = struct.unpack(">HH", frame[2:6])
        if fc in (3, 4):
            regs = self.hr if fc == 3 else self.ir
            if all(a in regs for a in range(addr, addr + val)):
                data = b"".join(struct.pack(">H", regs[a]) for a in range(addr, addr + val))
                pdu = bytes([fc, len(data)]) + data
            else:
                pdu = bytes([fc | 0x80, 2])
        elif fc == 6 and addr in self.hr:
            if self.on_write:
                self.on_write()
            self.hr[addr] = val
            self.writes.append((addr, val))
            pdu = frame[1:6]
        else:
            pdu = bytes([fc | 0x80, 2 if fc == 6 else 1])
        out = hs.with_crc(bytes([self.slave]) + pdu)
        if self.garble:
            self.garble -= 1
            out = out[:-1] + bytes([out[-1] ^ 0xFF])
        if self.noise:
            self.noise -= 1
            out = b"\x00" + out
        return out


class ShellTestCase(unittest.TestCase):
    """Base of the tests: simulated Flow40, temporary state and log files, shorter bus listening."""

    def setUp(self):
        """Prepares a temporary directory for state and log, shortens bus listening and starts the Flow40."""
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.tmp = tmp.name
        for name, value in (("STATE_FILE", os.path.join(tmp.name, "state.json")),
                            ("WRITE_LOG", os.path.join(tmp.name, "writes.log")),
                            ("BUS_LISTEN_OPEN", 0.05),
                            ("BUS_LISTEN_WRITE", 0.05)):
            patcher = mock.patch.object(hs, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)
        # On a pty, tcdrain waits until the simulator thread has read the request, and
        # older Python versions keep the GIL during that wait: the two threads would
        # deadlock. The bytes are already in the pty buffer, so skipping it changes nothing.
        patcher = mock.patch.object(hs.termios, "tcdrain", lambda fd: None)
        patcher.start()
        self.addCleanup(patcher.stop)
        self.unit = Flow40()
        self.addCleanup(self.unit.close)

    def run_cmd(self, *argv, stdin="", port=True):
        """Runs main() like './helty --port <pty> argv...' and returns (exit code, output).

        stdin provides the answers to confirmations. With port=False the port is not
        given and the shell picks it by itself.
        """
        out = io.StringIO()
        prefix = ["--port", self.unit.path] if port else []
        with mock.patch("sys.stdin", io.StringIO(stdin)), contextlib.redirect_stdout(out):
            rc = hs.main([*prefix, *argv])
        return rc, out.getvalue()

    def shell(self, *argv, open_port=True):
        """Creates a HeltyShell on the pty; with open_port the port is already open, as after start-up."""
        sh = hs.HeltyShell(hs.parse_args(["--port", self.unit.path, *argv]))
        if open_port:
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertTrue(sh.open_port(quiet=True))
        self.addCleanup(sh.close_port)
        return sh

    def onecmd(self, sh, line):
        """Runs one line in shell sh with an empty stdin and returns the output."""
        out = io.StringIO()
        with mock.patch("sys.stdin", io.StringIO("")), contextlib.redirect_stdout(out):
            sh.onecmd(line)
        return out.getvalue()

    def interactive(self, lines, sh=None):
        """Runs cmdloop as from a terminal with the given lines and returns the output.

        A KeyboardInterrupt item in lines simulates Ctrl+C at the prompt. readline is
        not configured, so the real history is left alone.
        """
        sh = sh or self.shell(open_port=False)
        out = io.StringIO()
        with mock.patch.object(hs, "_setup_readline"), \
                mock.patch("builtins.input", side_effect=lines), \
                contextlib.redirect_stdout(out):
            sh.cmdloop()
        return out.getvalue()


class TestReadings(ShellTestCase):
    """Protocol and basic readings."""

    def test_crc_known_vector(self):
        """The CRC of 01 03 00 00 00 0A is C5 CD, the Modbus reference value."""
        self.assertEqual(hs.with_crc(bytes.fromhex("01030000000A"))[-2:], bytes.fromhex("C5CD"))

    def test_status(self):
        """'status' identifies the unit and shows signed temperatures and no alarms."""
        rc, out = self.run_cmd("status")
        self.assertEqual(rc, 0, out)
        self.assertIn("Helty Flow40", out)
        self.assertIn("21.5 °C", out)
        self.assertIn("-3.0 °C", out)
        self.assertIn("none", out)


class TestSafeguards(ShellTestCase):
    """Write safeguards."""

    def test_minimum_interval_and_off(self):
        """A second write within WRITE_COOLDOWN is blocked, turning the unit off is not."""
        rc, out = self.run_cmd("speed", "2")
        self.assertEqual(rc, 0, out)
        rc, out = self.run_cmd("speed", "3")
        self.assertEqual(rc, 1)
        self.assertIn("too soon", out)
        rc, out = self.run_cmd("off")
        self.assertEqual(rc, 0, out)
        self.assertEqual(self.unit.writes, [(1000, 2), (1000, 0)])

    def test_minimum_interval_without_state_file(self):
        """If STATE_FILE is not writable the minimum interval still applies within the session, with a warning."""
        missing_dir = os.path.join(self.tmp, "missing", "state.json")
        with mock.patch.object(hs, "STATE_FILE", missing_dir):
            sh = self.shell()
            first = self.onecmd(sh, "speed 2")
            second = self.onecmd(sh, "speed 3")
        self.assertIn("applies to this session only", first)
        self.assertIn("too soon", second)
        self.assertEqual(self.unit.writes, [(1000, 2)])

    def test_registers_and_values_not_allowed(self):
        """Protected registers, registers outside WRITE_RULES, out-of-range values and read-only mode write nothing."""
        for argv, msg in ((("write", "5", "1"), "protected"),
                          (("write", "1001", "1"), "is not writable"),
                          (("write", "1000", "9"), "outside the allowed range"),
                          (("-r", "speed", "1"), "read-only")):
            with self.subTest(argv=argv):
                rc, out = self.run_cmd(*argv)
                self.assertEqual(rc, 1)
                self.assertIn(msg, out)
        self.assertEqual(self.unit.writes, [])


class TestNoise(ShellTestCase):
    """Corrupted responses on the bus."""

    def test_one_corrupted_frame_does_not_block_writes(self):
        """A wrong CRC during identification is overcome by reading the PID again."""
        sh = self.shell()
        self.unit.garble = 1
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertTrue(sh.identify())
        self.onecmd(sh, "speed 2")
        self.assertTrue(sh.last_ok)
        self.assertEqual(self.unit.writes, [(1000, 2)])

    def test_unreadable_pid_is_checked_again(self):
        """A PID unreadable at start-up is not cached: it is read again before the write."""
        sh = self.shell()
        self.unit.garble = hs.PID_ATTEMPTS        # every start-up attempt fails
        with contextlib.redirect_stdout(io.StringIO()) as out:
            self.assertFalse(sh.identify())
        self.assertIn("is unreadable", out.getvalue())
        self.onecmd(sh, "speed 2")               # the bus is clean now
        self.assertTrue(sh.last_ok)
        self.assertEqual(self.unit.writes, [(1000, 2)])

    def test_persistent_noise_blocks_writes(self):
        """With responses always corrupted the write is blocked."""
        sh = self.shell()
        self.unit.garble = 1000
        out = self.onecmd(sh, "speed 2")
        self.assertFalse(sh.last_ok)
        self.assertIn("Write blocked", out)
        self.assertEqual(self.unit.writes, [])

    def test_stray_byte_before_the_response(self):
        """A 0x00 byte before the response (typical of RS-485 direction switching) does not fail the command."""
        self.unit.noise = 1
        rc, out = self.run_cmd("temp")
        self.assertEqual(rc, 0, out)
        self.assertIn("21.5 °C", out)


class TestExitCode(ShellTestCase):
    """Exit code in single-command mode."""

    def test_start_cancelled_with_fault(self):
        """Start with an active fault and the confirmation refused: code 1, no write."""
        self.unit.ir[1006] = 0x5004               # internal fan fault
        rc, out = self.run_cmd("speed", "1")     # empty stdin: the confirmation means no
        self.assertEqual(rc, 1)
        self.assertIn("Command cancelled", out)
        self.assertEqual(self.unit.writes, [])

    def test_expert_write_cancelled(self):
        """Wrong expert-mode confirmation: code 1, no write."""
        self.unit.hr[1001] = 0                    # fictitious register, only for expert mode
        rc, out = self.run_cmd("--expert", "write", "1001", "5", stdin="1000\n")
        self.assertEqual(rc, 1)
        self.assertIn("Write cancelled", out)
        self.assertEqual(self.unit.writes, [])

    def test_mode_already_set(self):
        """Mode already set: code 0, no write."""
        self.unit.hr[1000] = 2
        rc, out = self.run_cmd("speed", "2")
        self.assertEqual(rc, 0, out)
        self.assertIn("already set", out)
        self.assertEqual(self.unit.writes, [])


class TestCtrlC(ShellTestCase):
    """Ctrl+C handling in the interactive shell."""

    def test_at_prompt_does_not_close_the_shell(self):
        """Ctrl+C at the prompt explains how to quit, the shell accepts more commands and start-up is not repeated."""
        out = self.interactive([KeyboardInterrupt, "temp", "exit"])
        self.assertIn("To quit type 'exit'", out)
        self.assertIn("21.5 °C", out)
        self.assertEqual(out.count("HeltyShell - Helty Flow40 control"), 1)

    def test_during_a_command_returns_to_prompt(self):
        """Ctrl+C during a command interrupts it and returns to the prompt."""
        sh = self.shell(open_port=False)
        with mock.patch.object(sh, "snapshot", side_effect=KeyboardInterrupt):
            out = self.interactive(["status", "exit"], sh)
        self.assertIn("Command interrupted", out)

    def test_at_confirmation_means_no(self):
        """Ctrl+C at a confirmation prompt cancels the command without writing."""
        self.unit.ir[1006] = 0x5004               # internal fan fault
        sh = self.shell()
        with mock.patch("builtins.input", side_effect=KeyboardInterrupt):
            out = self.onecmd(sh, "speed 1")
        self.assertIn("Command cancelled", out)
        self.assertFalse(sh.last_ok)
        self.assertEqual(self.unit.writes, [])

    def test_does_not_interrupt_a_write_in_progress(self):
        """A SIGINT arriving while the unit receives the write does not prevent read-back and log."""
        sh = self.shell()
        self.unit.on_write = lambda: os.kill(os.getpid(), signal.SIGINT)
        try:
            out = self.onecmd(sh, "speed 2")
        except KeyboardInterrupt:
            self.fail("Ctrl+C interrupted the write before read-back and log")
        self.assertIn("Mode: Speed 2", out)
        with open(hs.WRITE_LOG, encoding="utf-8") as f:
            self.assertIn("result=ok", f.read())


class TestMonitor(ShellTestCase):
    """The monitor command."""

    def test_unreadable_alarms(self):
        """With IR 1006 unreadable the monitor shows "alarms n/a" and not "no alarms"."""
        del self.unit.ir[1006]
        sh = self.shell()
        real, calls = sh.snapshot, []

        def one_round():
            """First call: real snapshot; second: Ctrl+C, which stops the monitor."""
            if calls:
                raise KeyboardInterrupt
            calls.append(True)
            return real()

        with mock.patch.object(sh, "snapshot", side_effect=one_round):
            out = self.onecmd(sh, "monitor 1")
        self.assertIn("alarms n/a", out)
        self.assertNotIn("no alarms", out)


class TestRememberedPort(ShellTestCase):
    """Port choice with several serial devices connected."""

    def setUp(self):
        """Adds a second pty, served by nobody, to the results of find_ports."""
        super().setUp()
        master, slave = os.openpty()             # another board plugged in, for example an ESP32
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        self.other = os.ttyname(slave)
        patcher = mock.patch.object(hs, "find_ports", return_value=[self.other, self.unit.path])
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_without_remembered_port_asks_which_one(self):
        """Without a remembered port the shell lists the ports and exits with 1."""
        rc, out = self.run_cmd("status", port=False)
        self.assertEqual(rc, 1)
        self.assertIn("Several serial ports found", out)

    def test_uses_the_port_where_the_unit_was_found(self):
        """After a successful identification the port is remembered and picked automatically."""
        rc, out = self.run_cmd("status")          # the first time the port must be given
        self.assertEqual(rc, 0, out)
        rc, out = self.run_cmd("status", port=False)
        self.assertEqual(rc, 0, out)
        self.assertIn(f"using {self.unit.path}", out)
        self.assertIn("21.5 °C", out)


class TestPort(ShellTestCase):
    """Port opening and completion."""

    def test_exclusive_port(self):
        """A second opening of the same port fails with EBUSY; once the first is closed, it opens again."""
        first = hs.SerialPort(self.unit.path)
        try:
            with self.assertRaises(OSError) as ctx:
                hs.SerialPort(self.unit.path)
            self.assertEqual(ctx.exception.errno, errno.EBUSY)
            rc, out = self.run_cmd("status")
            self.assertEqual(rc, 1)
            self.assertIn("already open in another program", out)
        finally:
            first.close()
        hs.SerialPort(self.unit.path).close()

    def test_tab_completes_the_port_path(self):
        """Tab on 'port' completes the path even though readline splits the word on '/' and '-'."""
        sh = hs.HeltyShell(hs.parse_args([]))
        with mock.patch.object(hs, "find_ports", return_value=["/dev/cu.usbserial-A10K"]):
            # the arguments mimic what readline passes: text is the piece after the last delimiter
            self.assertEqual(sh.complete_port("cu.usbs", "port /dev/cu.usbs", 10, 17),
                             ["cu.usbserial-A10K"])
            self.assertEqual(sh.complete_port("A", "port /dev/cu.usbserial-A", 23, 24),
                             ["A10K"])
            self.assertEqual(sh.complete_port("", "port ", 5, 5),
                             ["/dev/cu.usbserial-A10K"])


if __name__ == "__main__":
    unittest.main()
