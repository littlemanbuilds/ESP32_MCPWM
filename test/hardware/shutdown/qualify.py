"""Strict UART scoring for the USB-only ESP32-S3 shutdown fixture."""
import argparse
from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re
from statistics import mean
from time import sleep


PERIOD = 4000
FREQUENCY = 20000
CLOCK = 80_000_000
TICK_MASK = (1 << 32) - 1
PINS = ((15, 4), (16, 6), (17, 2))
KINDS = ("coast", "stop", "fault_coast")


def require(condition, detail):
    if not condition:
        raise ValueError(detail)


@dataclass(frozen=True)
class Case:
    id: int
    duty: int
    kind: int
    offset: int
    route: int

    def command(self):
        return f"{self.id} {self.duty} {self.kind} {self.offset} {self.route}\n".encode()


def cases():
    # Sweep before the operation, then score the phase of the actual EN edge.
    # Reverse trials start from isolated quiet after the forward sweep.
    primary = [(duty, kind, offset, 0) for duty in (15, 40)
               for kind in range(3) for offset in range(0, 50, 2)]
    reverse = [(duty, kind, offset, 1) for duty in (15, 40)
               for kind in range(3) for offset in (0, 16, 32)]
    return [Case(i, *setting) for i, setting in enumerate(primary + reverse)]


@dataclass(frozen=True)
class Window:
    channel: tuple
    request_us: int
    returned_us: int


class Trace:
    def __init__(self, connection, raw):
        self.connection = connection
        self.raw = raw

    def line(self):
        line = self.connection.readline()
        self.raw.write(line)
        self.raw.flush()
        require(line.endswith(b"\n"), f"Missing or incomplete UART line {line!r}")
        return line.decode("ascii").rstrip("\r\n")

    def expect(self, expected):
        observed = self.line()
        require(observed == expected, f"Expected {expected!r}, got {observed!r}")

    def identity(self, elf):
        # A cold ROM load can precede fixture records. A later boot is a failure.
        rom_types = (
            r"ESP-ROM:esp32s3-[0-9]+", r"Build:.*", r"rst:0x.*",
            r"Saved PC:0x[0-9a-f]+", r"SPIWP:0x[0-9a-f]+", r"mode:.*",
            r"load:0x[0-9a-f]+,len:.*", r"entry 0x[0-9a-f]+", r"",
        )
        for _ in range(20):
            line = self.line()
            if line.startswith("IDENTITY "):
                require(line == f"IDENTITY {elf} {CLOCK}", f"Wrong fixture identity or clock {line!r}")
                return
            require(any(re.fullmatch(pattern, line) for pattern in rom_types),
                    f"Unexpected startup UART record {line!r}")
        raise ValueError("No fixture identity")

    def window(self, name, identifier):
        line = self.line()
        fields = line.split()
        require(len(fields) == 14 and fields[:3] == ["WINDOW", name, str(identifier)],
                f"Malformed or misplaced window {line!r}")
        count, overflow, levels, request, returned, *values = map(int, fields[3:])
        rises, statuses = values[:3], values[3:]
        require(0 <= count <= 96 and overflow == levels == 0 and statuses == [0, 0, 0],
                f"{name} {identifier} has overflow, nonzero final level or counter error")
        require(0 <= request <= TICK_MASK and 0 <= returned <= TICK_MASK,
                f"{name} {identifier} request timestamps outside uint32")
        channel = [[], [], []]
        for _ in range(count):
            edge = self.line().split()
            require(len(edge) == 4 and edge[0] == "EDGE", f"Malformed edge {edge!r}")
            tick, index, rising = map(int, edge[1:])
            require(0 <= tick <= TICK_MASK and 0 <= index < 3 and rising in (0, 1),
                    f"Invalid capture edge {edge!r}")
            channel[index].append((tick, rising))
        self.expect("END")
        # Each input starts and finishes low. Require alternating rising/falling
        # edges and an independent PCNT rise count for every input.
        for i, events in enumerate(channel):
            require(len(events) == 2 * rises[i], f"{name} {identifier} count mismatch on {i}")
            for j, (tick, rising) in enumerate(events):
                require(rising == (1 - j % 2), f"{name} {identifier} lost or inverted edge on {i}")
                if j:
                    delta = (tick - events[j - 1][0]) & TICK_MASK
                    require(0 < delta < CLOCK // 10, f"{name} {identifier} invalid timestamps on {i}")
        return Window(tuple(channel), request, returned)


def ticks_between(a, b):
    return ((a - b + (1 << 31)) & TICK_MASK) - (1 << 31)


def score(case, window):
    pwm = window.channel[case.route]
    require(not window.channel[1 - case.route], f"Case {case.id} inactive PWM route toggled")
    enable = window.channel[2]
    require(len(enable) == 2, f"Case {case.id} needs exactly one EN activation and withdrawal")
    on, off = (edge[0] for edge in enable)
    require(0 < ticks_between(off, on) < CLOCK // 100, f"Case {case.id} EN interval invalid")
    request_duration = (window.returned_us - window.request_us) & TICK_MASK
    require(request_duration < 10_000, f"Case {case.id} shutdown did not return promptly")
    before = [ticks_between(tick, off) for tick, polarity in pwm if polarity and ticks_between(tick, off) < 0]
    after = [ticks_between(tick, off) for tick, polarity in pwm if polarity and ticks_between(tick, off) >= 0]
    require(len(before) >= 7 and len(after) <= 1 and max(after, default=0) <= PERIOD,
            f"Case {case.id} PWM aperture violation or insufficient baseline: {after!r}")
    require(ticks_between(pwm[0][0], on) >= 0, f"Case {case.id} PWM rose before fresh enable")
    carrier = [b - a for a, b in zip(before, before[1:])]
    require(all(abs(value - PERIOD) <= 8 for value in carrier),
            f"Case {case.id} measured period changed: {carrier!r}")
    complete = [(ticks_between(pwm[i + 1][0], pwm[i][0]), ticks_between(pwm[i + 1][0], off))
                for i in range(0, len(pwm), 2)]
    width = [length for length, falling in complete if falling < -PERIOD]
    nominal_width = {15: 560, 40: 1600}[case.duty]
    require(len(width) >= 4 and all(abs(value - nominal_width) <= 8 for value in width),
            f"Case {case.id} unexpected measured pulse widths: {width!r}")
    phase = (-before[-1]) % PERIOD
    final_fall = ticks_between(pwm[-1][0], off)
    require(final_fall <= PERIOD + nominal_width + 8, f"Case {case.id} final pulse did not terminate")
    return {
        "id": case.id, "route": case.route, "operation": KINDS[case.kind], "command_percent": case.duty,
        "configure_hz": FREQUENCY, "measured_hz": CLOCK / mean(carrier),
        "measured_percent": 100 * mean(width) / mean(carrier), "phase_bin": phase // 500,
        "phase_ticks": phase, "enable_fall_ticks": off, "request_us": window.request_us,
        "return_us": window.returned_us, "post_enable_rises": after, "final_fall_ticks": final_fall,
        "high_at_enable_fall": any(ticks_between(pwm[i][0], off) < 0 <= falling
                                  for i, (_, falling) in zip(range(0, len(pwm), 2), complete)),
    }


def quiet(window, label):
    require(not any(window.channel) and window.request_us == window.returned_us == 0,
            f"{label} was not quiet")


def qualify(connection, raw, elf, record):
    trace = Trace(connection, raw)
    trace.identity(elf)
    for index, (output, input_pin) in enumerate(PINS):
        trace.expect(f"INIT {index} {output} {input_pin} 0 0 0 0 0 0 1 0")
    for index, polarity in enumerate((2, 1, 3)):
        trace.expect(f"CAPTURE {index} {polarity} 0 0")
    quiet(trace.window("idle", 0), "initial idle")
    for witness in range(3):
        if witness < 2:
            trace.expect(f"ROUTE {witness} 0 0 0")
        edges = trace.window("witness", witness).channel
        require([len(items) for items in edges] == [2 * (i == witness) for i in range(3)],
                f"Wiring witness {witness} missing or crossed")
        pulse = ticks_between(edges[witness][1][0], edges[witness][0][0])
        require(CLOCK // 30_000 <= pulse <= CLOCK // 10_000, f"Witness {witness} pulse invalid")
    trace.expect("MOTOR 0 1")
    quiet(trace.window("preflight", 0), "motor preflight")
    trace.expect("READY")
    results = []
    for case in cases():
        connection.write(case.command())
        trace.expect(f"ROUTE {case.route} 0 0 0")
        row = score(case, trace.window("shutdown", case.id))
        trace.expect("RECOVER")
        connection.write(b"recover\n")
        quiet(trace.window("recovery", case.id), "recovery without fresh drive")
        trace.expect("READY")
        results.append(row)
        record.write(json.dumps(row) + "\n")
        record.flush()
    connection.write(b"finish\n")
    quiet(trace.window("final", len(results)), "final inactive state")
    trace.expect("DONE")
    for duty in (15, 40):
        for operation in KINDS:
            cohort = [row for row in results if row["route"] == 0 and
                      row["command_percent"] == duty and row["operation"] == operation]
            require({row["phase_bin"] for row in cohort} == set(range(8)),
                    f"{duty}% {operation} has incomplete phase coverage")
            require(any(row["high_at_enable_fall"] for row in cohort),
                    f"{duty}% {operation} never withdrew EN during an active PWM pulse")
    return {
        "transitions": len(results), "primary_transitions": sum(row["route"] == 0 for row in results),
        "max_post_enable_rises": max(len(row["post_enable_rises"]) for row in results),
        "worst_aperture_us": max(max(row["post_enable_rises"], default=0) for row in results) / 80,
        "last_fall_us": max(row["final_fall_ticks"] for row in results) / 80,
        "frequency_range_hz": [min(row["measured_hz"] for row in results),
                               max(row["measured_hz"] for row in results)],
        "duty_range_percent": {duty: [
            min(row["measured_percent"] for row in results if row["command_percent"] == duty),
            max(row["measured_percent"] for row in results if row["command_percent"] == duty),
        ] for duty in (15, 40)},
    }


def verify_bundle(bundle, expected_profile):
    manifest = json.loads((bundle / "manifest.json").read_text())
    require(manifest["profile"] == expected_profile, "Wrong bundle profile")
    require(set(manifest["regions"]) == {"bootloader.bin", "partitions.bin", "boot_app0.bin", "firmware.bin"},
            "Flash region inventory differs")
    end = 0
    for name, region in sorted(manifest["regions"].items(), key=lambda item: item[1]["offset"]):
        payload = (bundle / name).read_bytes()
        require(len(payload) == region["size"] and hashlib.sha256(payload).hexdigest() == region["sha256"],
                f"Modified image region {name}")
        require(region["offset"] >= end, f"Overlapping flash region {name}")
        end = region["offset"] + len(payload)
    require(end <= 16 * (1 << 20), "Firmware scope exceeds flash capacity")
    elf = hashlib.sha256((bundle / "firmware.elf").read_bytes()).hexdigest()
    require((bundle / "firmware.bin").read_bytes()[0xB0:0xD0].hex() == elf == manifest["elf_sha256"],
            "Embedded and linked ELF identities disagree")
    return manifest


def main():
    import serial

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("profile", choices=("legacy", "current"))
    parser.add_argument("bundle", type=Path)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    manifest = verify_bundle(args.bundle, args.profile)
    args.session.mkdir()
    (args.session / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    connection = serial.Serial(baudrate=115200, timeout=5, write_timeout=1, exclusive=True)
    connection.dtr = connection.rts = False
    connection.port = args.port
    with (args.session / "uart.raw").open("xb") as raw, (args.session / "results.jsonl").open("x") as record:
        try:
            connection.open()
            connection.rts = True
            sleep(0.1)
            connection.rts = False
            summary = qualify(connection, raw, manifest["elf_sha256"], record)
        except (ValueError, OSError) as failure:
            (args.session / "failure.txt").write_text(str(failure) + "\n")
            raise
        finally:
            connection.close()
    (args.session / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
