"""Regenerate perf-script.txt, the fixture the flamegraph parser is tested against.

Needs perf and a matching linux-tools, plus CAP_PERFMON (or a lowered
perf_event_paranoid). Run inside the benchmark container:

    python3 make_perf_script.py
"""

import subprocess
import sys
import textwrap
from pathlib import Path

WORKLOAD = textwrap.dedent("""
    import time
    def leaf(t):
        end = time.monotonic() + t
        while time.monotonic() < end:
            pass
    def middle(t): leaf(t)
    def outer(t): middle(t)
    outer(0.7)
""")


def main() -> int:
    """Record a short profile and write its perf script output."""
    Path("w.py").write_text(WORKLOAD)
    subprocess.run(
        [
            "perf",
            "record",
            "-q",
            "-F",
            "199",
            "-g",
            "-o",
            "p.data",
            "--",
            "python3",
            "w.py",
        ],
        check=True,
    )
    out = subprocess.run(
        ["perf", "script", "-i", "p.data"], check=True, capture_output=True, text=True
    ).stdout
    Path("perf-script.txt").write_text(out)
    print(f"wrote {len(out.splitlines())} lines; trim before committing")
    return 0


if __name__ == "__main__":
    sys.exit(main())
