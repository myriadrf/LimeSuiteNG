#!/usr/bin/env bash
# LimeSuiteNG board check. Runs each step in order and stops at the
# first failure, so the last printed step is the one that failed.
# Exit code 0 means the board passed. See RUNBOOK.md for details.

set -e
trap 'echo "FAIL"' ERR

echo "1. Board enumerates"
limeDevice

echo "2. Identity readable"
limeDevice --full

echo "3. Configuration accepted: 10 MHz sample rate, 1 GHz LO, Rx test signal"
limeConfig --initialize --samplerate 10e6 --rxen 1 --rxlo 1e9 --rxtestsignal 1

echo "4. Rx streaming works: 2 seconds through the whole digital path"
limeTRX --time 2000

echo "PASS"
