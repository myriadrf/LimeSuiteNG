"""Shared building blocks for board manufacturing tests.

- operator:    console interface to the human operator (instructions, manual value entry)
- instruments: generic signal generator / spectrum analyzer interfaces, with SCPI and manual (operator) drivers
- fixture:     DUT port connections, done by the operator or by an RF switch command
- limetools:   wrappers around the LimeSuiteNG CLI tools (limeDevice, limeConfig, limeTRX)
- dsp:         sample capture analysis
- report:      test results, console summary, CSV/JSON output
- station:     test station configuration (instruments, fixture, path losses)
"""
