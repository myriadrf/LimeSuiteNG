"""Console interface to the human operator running the test station."""

import sys


class OperatorAbort(Exception):
    """The operator chose to abort the test run."""


class Operator:
    """Shows instructions and collects manually measured values.

    All prompts go to stderr, so stdout stays free for machine readable output.
    Entering 'q' at any prompt aborts the run (raises OperatorAbort).
    """

    def __init__(self, stream_in=None, stream_out=None):
        self.stream_in = stream_in or sys.stdin
        self.stream_out = stream_out or sys.stderr

    def _print(self, text=""):
        print(text, file=self.stream_out, flush=True)

    def _readline(self, prompt):
        self.stream_out.write(prompt)
        self.stream_out.flush()
        line = self.stream_in.readline()
        if not line:  # EOF, nobody to answer
            raise OperatorAbort("no operator input (end of input)")
        line = line.strip()
        if line.lower() == "q":
            raise OperatorAbort("aborted by operator")
        return line

    def info(self, text):
        self._print(text)

    def instruct(self, text):
        """Asks the operator to do something and waits until it is done."""
        self._print()
        self._print(">>> OPERATOR ACTION: " + text)
        self._readline("    Press ENTER when done (q to abort) ")

    def ask_float(self, text, unit=""):
        """Asks the operator for a measured value. Returns the value as float."""
        self._print()
        self._print(">>> OPERATOR MEASUREMENT: " + text)
        suffix = " [" + unit + "]" if unit else ""
        while True:
            line = self._readline("    Enter value" + suffix + " (q to abort): ")
            try:
                return float(line)
            except ValueError:
                self._print("    Not a number: '" + line + "'")

    def confirm(self, text):
        """Asks a yes/no question."""
        while True:
            line = self._readline(">>> " + text + " [y/n] ").lower()
            if line in ("y", "yes"):
                return True
            if line in ("n", "no"):
                return False
