<p align="center">
  <img src="assets/icon.svg" alt="HeltyShell icon" width="96">
</p>

<h1 align="center">
  <img src="assets/name.svg" alt="HeltyShell" width="360">
</h1>

<p align="center">
  <strong>Interactive shell for a Helty Flow40 ventilation unit over Modbus RTU.<br>
  Read temperatures and alarms, change the fan speed, inspect any register. One Python file, no dependencies.</strong>
</p>

<p align="center">
  <a href="https://www.python.org/"><img alt="Python 3.9+" src="https://img.shields.io/badge/Python-3.9%2B-3776AB?logo=python&logoColor=fff"></a>
  <img alt="macOS | Linux" src="https://img.shields.io/badge/Platform-macOS%20%7C%20Linux-333">
  <img alt="Modbus RTU" src="https://img.shields.io/badge/Modbus-RTU-333">
  <img alt="No dependencies" src="https://img.shields.io/badge/Dependencies-none-333">
  <a href="LICENSE"><img alt="MIT" src="https://img.shields.io/badge/License-MIT-333"></a>
</p>


HeltyShell talks to a **Helty Flow40**, a single-room ventilation unit with heat recovery, through the Modbus terminal block of the unit and an inexpensive **USB-RS485 adapter**. It finds the unit on the bus by itself, shows its state in plain words and lets you change the operating mode from the terminal.

It is meant for the moments when you need to look inside the unit: checking the wiring, finding its Modbus address, exploring its registers or trying commands before building a permanent integration.

Writing to a machine that ventilates your home deserves some care, so every write goes through a single path with a dozen safeguards. Out of the box only the machine state register is writable, the unit's model is verified before anything is sent, and a write that is not acknowledged is never retried blindly.

The whole tool is a single Python file that drives the serial port directly through `termios`: no `pyserial`, no `pip install`, just the `python3` that already comes with macOS.

---

## Getting started

### Requirements

- A **Helty Flow40** with the Modbus terminal block: the green 7-position terminal block (1-6 and X) next to the RJ socket.
- A **USB-RS485 adapter**, such as the common CH340/CH343, FTDI or CP210x based ones. The shell finds it by itself when it shows up as `/dev/cu.usbserial*`, `/dev/cu.usbmodem*`, `/dev/cu.wchusbserial*`, `/dev/ttyUSB*` or `/dev/ttyACM*`; any other path can be given with `--port`.
- **macOS or Linux** with **Python 3.9** or later.

### Wiring

Connect the adapter to terminals 4 and 5 of the unit:

| Flow40 terminal | Signal | Adapter |
|---|---|---|
| 4 | Modbus + | A (D+) |
| 5 | Modbus - | B (D-) |

The adapter is powered over USB, so the 24 V terminals of the unit (1 and 2) stay unconnected. Turn the unit off before opening it and connecting the wires.

If the unit does not respond, swap A and B: manufacturers do not label the two lines consistently, and swapping them does no harm.

The RS-485 bus must have **a single master**. If another Modbus gateway is connected to the unit, for example an ESPHome device, disconnect it while HeltyShell is in use.

### Running

```bash
git clone https://github.com/fabioscarparo/HeltyShell.git
cd HeltyShell
./helty
```

The shell finds the serial port and the unit by itself. A single command can also be run without opening the shell, which makes HeltyShell usable from scripts:

```bash
./helty status
./helty speed 2
```

The exit code is `0` only if the command succeeded: a command blocked by a safeguard or cancelled at a confirmation returns `1`.

---

## At a glance

```text
$ ./helty
HeltyShell - Helty Flow40 control over Modbus RTU
Port: /dev/cu.usbserial-110 · 19200 8N1
✓ Unit found: slave 2, Helty Flow40 (PID 0x1102)
Type 'help' for the list of commands, 'exit' to quit.
helty[2]> status
  Mode                Off  (0)
  Indoor temp.        24.9 °C
  Outdoor temp.       23.2 °C
  Alarms              none
helty[2]> speed 2
✓ Mode: Speed 2
helty[2]> mode
    0  Off
    1  Speed 1
  ● 2  Speed 2
    3  Speed 3
    4  Speed 4
    5  Hyper
    6  Night
    7  Free cooling
helty[2]> write 5 1
Write blocked: HR 5 is protected (password and non-volatile data): the shell never writes it.
```

---

## Commands

### Control

| Command | Description |
|---|---|
| `status` | Mode, temperatures and alarms |
| `off` | Turns the unit off |
| `speed <1-4>` | Sets the fan speed |
| `hyper`, `night`, `freecooling` | Start a preset |
| `mode [0-7 \| name]` | Without arguments shows the current mode and the list, otherwise sets it |
| `monitor [seconds]` | Reads the status at intervals (default 5 s), Ctrl+C to stop |

Mode names also work as commands on their own: `v1` to `v4`, `boost` for Hyper, `free` or `fc` for Free cooling.

### Readings

| Command | Description |
|---|---|
| `temp` | Indoor and outdoor temperature |
| `alarms` | The alarm register, bit by bit |
| `info` | Port, slave, firmware and model |
| `dump [ir\|hr] [from] [to]` | Lists registers one by one (default IR 1000-1020) |
| `read <ir\|hr> <address> [count]` | Reads raw registers, in decimal, hex and signed |
| `write <address> <value>` | Writes a holding register, through every safeguard below |

### Unit and bus

| Command | Description |
|---|---|
| `identify` | Finds the unit on the bus and checks its model |
| `scan [from] [to]` | Lists every slave that responds and its model (default 1-247) |
| `id [n]` | Shows or changes the slave address, checking the model |
| `port [path]` | Shows the current and available ports, or opens another one |
| `timeout [seconds]` | Shows or changes the response timeout |
| `debug [on\|off]` | Shows the Modbus frames sent and received |

### Safety

| Command | Description |
|---|---|
| `safety` | Summary of the active safeguards |
| `readonly [on\|off]` | Blocks or allows every write |
| `log [n]` | The latest writes from the log (default 10) |
| `exit` | Closes the shell (also `quit` or Ctrl+D) |

`help` lists the commands and `help <command>` explains one. Tab completes commands, mode names and port paths, and the history is kept between sessions.

Ctrl+C interrupts the running command or clears the line being typed without closing the shell, and at a confirmation prompt it means *no*. A write that has already been sent is not interrupted: the shell reads it back and logs it before returning to the prompt.

---

## Safeguards

Every write goes through a single check, in this order:

1. **Read-only mode.** With `--read-only` or `readonly on`, no write is sent.
2. **Protected registers.** HR 0-15 hold the password and non-volatile data and are never written, not even in expert mode.
3. **Allowed registers and values.** Without `--expert`, only HR 1000 (machine state) is writable, with values 0 to 7.
4. **Model check.** Before writing, the shell verifies that the slave is a Flow40 (PID `0x1102`) and blocks otherwise; `--ignore-pid` allows other models only when you ask for it. A corrupted response is requested again, so a single glitch on the bus does not block the session.
5. **No needless writes.** If the register already holds the requested value, nothing is sent.
6. **Minimum interval.** At least 3 seconds between two writes, also across separate runs of `./helty`. Turning the unit off is always allowed.
7. **Typed confirmation in expert mode.** Writing a register other than HR 1000 requires typing its number.
8. **Free bus.** The shell listens to the bus for 0.3 seconds before writing and blocks the write if it hears another master. This is a best-effort check: a master that polls every 30 or 60 seconds almost always goes unnoticed, so disconnecting it is what really protects the bus.
9. **Active faults.** If the unit reports a fault (sensors, fans, EEPROM, frost), starting it requires a confirmation.
10. **No retries.** If the unit does not acknowledge a write, the shell does not repeat it: it reads the register back and reports its current value.
11. **Read-back and log.** Every write is read back and logged to `helty_writes.log` with the previous, requested and read-back values.
12. **No broadcast.** Every request goes to a single address (1-247), never to all units at once.
13. **Exclusive port.** While the shell is open, no other HeltyShell can use the same serial port, and on real adapters no other program can either.

Reading never changes anything: `status`, `monitor`, `dump`, `read` and `scan` are always safe.

---

## Options

| Option | Default | Description |
|---|---|---|
| `--port`, `-p` | detected automatically | Serial port |
| `--id`, `-a` | found automatically | Slave address of the unit (1-247) |
| `--baud`, `-b` | `19200` | Baud rate |
| `--parity`, `-P` | `N` | Parity: `N`, `E` or `O` |
| `--timeout`, `-t` | `0.5` | Response timeout in seconds |
| `--debug`, `-d` | off | Shows the Modbus frames |
| `--read-only`, `-r` | off | Blocks every write |
| `--expert` | off | Allows writing registers other than HR 1000, with a typed confirmation |
| `--ignore-pid` | off | Allows writes even if the model is not recognised |

The exit codes are `0` on success, `1` when the command fails, is blocked or is cancelled, `2` for an invalid `--id` and `130` when interrupted with Ctrl+C. Colours are turned off when the output is not a terminal or the `NO_COLOR` variable is set.

---

## How it works

### Talking to the unit

The Flow40 speaks the **Helty Device Remote Protocol** (HDRP) and the **Helty Device Application Protocol** (HDAP), built on Modbus RTU:

| Parameter | Value |
|---|---|
| Physical layer | RS-485, terminals 4 (+) and 5 (-) |
| Speed and format | 19200 baud, 8 data bits, no parity, 1 stop bit |
| Factory address | 2 |
| Functions used | `03` read holding registers, `04` read input registers, `06` write single register |

### Registers

The map below was read from a Flow40 with firmware 117 (PID `0x1102`). The shell shows these descriptions in `dump` and `read`.

| Register | Content | Notes |
|---|---|---|
| HR 1000 | Machine state | 0 off, 1-4 speed, 5 Hyper, 6 Night, 7 Free cooling |
| HR 0-15 | Password and non-volatile data | Never written |
| IR 1000 | Indoor temperature | Signed tenths of °C |
| IR 1001 | Outdoor temperature | Signed tenths of °C, below zero in winter |
| IR 1005 | CO2 (ppm) | No sensor on the Flow40 |
| IR 1006 | Alarms | Bitmask, see below |
| IR 1010 | Unknown | Maybe an hour counter; same value as IR 1020 |
| IR 1011 | Maybe a temperature | Tenths of °C? |
| IR 1012 | Firmware version | |
| IR 1013 | Product ID | `0x1102` for the Flow40 |
| IR 1015 | VOC (ppm) | No sensor on the Flow40 |

The alarm register (IR 1006) uses one bit per condition:

| Bit | Meaning |
|---|---|
| `0x0001` | Internal sensor fault |
| `0x0002` | External sensor fault |
| `0x0004` | Internal fan fault |
| `0x0008` | External fan fault |
| `0x0010` | EEPROM error |
| `0x0020` | Frost alarm |
| `0x0040`, `0x0080` | High and low humidity (require the humidity sensor) |
| `0x0100` | Temperature delta |
| `0x0200` | CO2 alarm (requires the CO2 sensor) |
| `0x0400` | Filter alarm |
| `0x1000` | Humidity sensor missing |
| `0x4000` | CO2 sensor missing |

On a Flow40 the last two bits are always set, because the unit has neither sensor: a healthy unit reads `0x5000` (20480), and `status` does not list those two bits as alarms.

### Finding the unit

At start-up the shell tries the last address that worked on that port, then the factory address 2. If neither responds it scans the whole bus (1-247, about 40 seconds) and checks the product ID of every device that answers. When it finds a Flow40 it remembers the port and the address, so the next start is immediate.

With several serial devices connected, for example the adapter and a development board, it uses the port where the unit was found last time. It never tries the other ports at random: opening a port can reset boards such as an ESP32.

### Driving the serial port

The port is opened in raw mode through `termios`, with a non-blocking file descriptor and `select()` for timeouts. Responses are framed by their expected length, and each one is checked for CRC, sender address, function code and byte count before it is used. A 20 ms gap separates two frames.

The port is locked with `flock` against other instances of the shell and with `TIOCEXCL` against any other program, so two masters cannot end up sharing the same adapter.

---

## Files

The shell keeps its files next to the script:

| File | Content |
|---|---|
| `.helty_state.json` | Remembered port and address, time of the last write |
| `.helty_history` | Command history |
| `helty_writes.log` | One line per write attempt |

---

## Tests

The tests run against a **Flow40 simulated on a pseudo-terminal**, so neither the unit nor the adapter is needed:

```bash
python3 -m unittest -v test_helty_shell
```

The simulated unit can corrupt its responses on request, which lets the tests cover the safeguards, noisy buses, exit codes, Ctrl+C, the remembered port, exclusive access and Tab completion. They use a temporary directory and never touch the real state and log files.

---

## Project structure

```text
assets/
  icon.svg               the HeltyShell icon
  name.svg               the HeltyShell wordmark

helty                    launcher: ./helty [options] [command]
helty_shell.py           the whole shell: serial port, Modbus RTU client, safeguards, commands
test_helty_shell.py      tests against a simulated Flow40
```

---

## Known limits

HeltyShell has been developed and tested on macOS with a Flow40 (firmware 117, PID `0x1102`) and a CH343-based USB-RS485 adapter. Linux uses the same `termios` interfaces and its device names are recognised, but it has not been tried with real hardware. Windows is not supported.

Only the Flow40 product ID is recognised. Other Helty units may speak the same protocol, but writes stay blocked unless the shell is started with `--ignore-pid`.

The register map comes from reading a real unit: some registers are still unknown, and their descriptions in the shell are marked as guesses.

---

## Note

HeltyShell is a **personal project**, provided as is: use it at your own risk. Opening the unit and connecting anything to it may affect its warranty.

It is **not affiliated with, endorsed by, or developed by Helty**. Helty and Flow40 are trademarks of their respective owners, used only to describe compatibility and the hardware used during development.
