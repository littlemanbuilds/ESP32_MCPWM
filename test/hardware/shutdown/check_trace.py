"""Compile the fixture's actual wire formatting and challenge the UART checker."""
import io
from pathlib import Path
import subprocess
import sys

from qualify import Case, Trace, Window, score


EMITTER = r"""
#include "trace.h"
int main()
{
    char line[128]{};
    const int16_t counts[] = {0, 1, 0};
    const int status[] = {0, 0, 0};
    formatWindow(line, sizeof(line), "probe", 23, 2, 0, 0, 654321U, 654333U, counts, status);
    std::puts(line);
    formatEdge(line, sizeof(line), UINT32_C(4294967280), 1, 1);
    std::puts(line);
    formatEdge(line, sizeof(line), 17, 1, 0);
    std::puts(line);
    std::puts("END");
}
"""


def rejected(operation, description):
    try:
        operation()
    except ValueError:
        return
    raise AssertionError(f"Accepted {description}")


def main():
    scratch = Path(sys.argv[1])
    compiler = sys.argv[2]
    source = scratch / "emitter.cpp"
    source.write_text(EMITTER)
    executable = scratch / "emitter"
    subprocess.run([compiler, "-std=c++11", "-Wformat=2", "-Werror=format",
                    "-I", Path(__file__).resolve().parent, source, "-o", executable], check=True)
    raw = subprocess.check_output([executable])
    copied = io.BytesIO()
    window = Trace(io.BytesIO(raw), copied).window("probe", 23)
    assert window == Window(([], [(4294967280, 1), (17, 0)], []), 654321, 654333)
    assert copied.getvalue() == raw
    for altered, description in (
        (raw[:-2], "truncated UART"),
        (raw.replace(b"7280 1 1", b"7280 1 0"), "incorrect edge polarity"),
        (raw.replace(b"654333 0 1 0", b"654333 0 2 0"), "independent count mismatch"),
        (raw.replace(b"23 2 0 0", b"23 2 1 0"), "capture overflow"),
        (raw.replace(b"23 2 0 0", b"23 2 0 1"), "active final output"),
        (raw.replace(b"654333", b"6543\xff3"), "corrupt UART byte"),
    ):
        rejected(lambda: Trace(io.BytesIO(altered), io.BytesIO()).window("probe", 23), description)
    case = Case(0, 15, 0, 0, 0)
    pre = tuple(edge for tick in range(4000, 36000, 4000) for edge in ((tick, 1), (tick + 560, 0)))
    en = ((1000, 1), (33500, 0))
    assert score(case, Window((pre, (), en), 800, 805))["post_enable_rises"] == []
    for bad_edges, description in (
        (((33800, 1), (34360, 0), (37500, 1), (38060, 0)), "two post-EN rises"),
        (((37600, 1), (38160, 0)), "post-EN rise beyond one period"),
    ):
        rejected(lambda: score(case, Window((pre + bad_edges, (), en), 800, 805)), description)
    print("Production wire formatter and strict UART checker: PASS")


if __name__ == "__main__":
    main()
