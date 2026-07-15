# Board check runbook

`board_check.sh` verifies that a connected LimeSDR board works with
this LimeSuiteNG build: it enumerates, reports its identity, accepts
configuration and streams samples. No antennas or signal generators
are needed, the streaming step uses the chip test signal. It stops at
the first failing step and ends with PASS or FAIL; the exit code is 0
only on PASS. Takes about a minute. Connect one board at a time.

## Prerequisites

1. LimeSuiteNG installed, so that `limeDevice`, `limeConfig` and
   `limeTRX` are in PATH.
2. PCIe boards (XTRX, X3): the limepcie kernel driver loaded,
   check with `lsmod | grep limepcie`.
3. USB boards (Mini, USB): udev rules installed
   (`udev-rules/install.sh`), then replug the board.

## Running

    tools/board-check/board_check.sh

## When a step fails

- Step 1: check cabling, `lsusb` or `lspci`, driver and udev rules.
  A board that never enumerates is a setup problem, not a software bug.
- Steps 2 to 4: rerun the failing command by hand with `--log debug`
  and attach its output plus `limeDevice --full` to an issue at
  https://github.com/myriadrf/LimeSuiteNG/issues

## Release candidate check

Before promoting a release candidate to final: install the rc
packages on a clean machine, run this script against a real board.
PASS: promote. FAIL: file the issue, fix, build new rc packages,
repeat.
