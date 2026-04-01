#!/usr/bin/env python3
"""
Field run helper for COM10/COM11/COM12 with focus on SEAL DONE stability.

Scenario:
1. Capture baseline statuses.
2. Send COMMON START on COM12.
3. Detect WaitDone on COM10 and prompt operator to press DONE button once.
4. Continue polling statuses and store logs.
5. Build report.txt + summary.json with key metrics.
"""

from __future__ import annotations

import argparse
import json
import re
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import serial


PORT_CONFIG = {
    "COM10": {"baud": 115200},
    "COM11": {"baud": 115200},
    "COM12": {"baud": 115200},
}

TS_FORMAT = "%Y-%m-%d %H:%M:%S.%f"

COMMON_PROGRESS_PATTERNS = (
    "COMMON: manipulator start",
    "COMMON TRACE: stage=wait_manip_ready",
    "COMMON TRACE: stage=wait_manip_step7",
    "COMMON TRACE: stage=wait_next_batch",
)


def now_iso() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def parse_ts(text: str) -> Optional[datetime]:
    try:
        return datetime.strptime(text, TS_FORMAT)
    except ValueError:
        return None


def safe_text(value: Optional[str]) -> str:
    return value if value else "<none>"


@dataclass
class LogLine:
    ts: str
    dt: Optional[datetime]
    text: str

    def full(self) -> str:
        return f"{self.ts} {self.text}"


class LiveSignals:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.wait_done_event = threading.Event()
        self.completion_event = threading.Event()
        self.first_wait_done_line: Optional[str] = None
        self.first_wait_done_ts: Optional[str] = None
        self.first_completion_line: Optional[str] = None
        self.first_completion_ts: Optional[str] = None
        self.first_completion_monotonic: Optional[float] = None

    def observe(self, port: str, ts: str, line: str, monotonic_ts: float) -> None:
        if port != "COM10":
            return

        wait_done_hit = ("SEAL: run_state=wait_done" in line) or ("SEAL WAIT_DONE:" in line)
        completion_hit = (
            "SEAL: cycle complete input became active." in line
            or "SEAL: cycle complete (synthetic DONE)." in line
        )

        with self.lock:
            if wait_done_hit and self.first_wait_done_line is None:
                self.first_wait_done_line = line
                self.first_wait_done_ts = ts
                self.wait_done_event.set()

            if completion_hit and self.first_completion_line is None:
                self.first_completion_line = line
                self.first_completion_ts = ts
                self.first_completion_monotonic = monotonic_ts
                self.completion_event.set()

    def get_first_completion_monotonic(self) -> Optional[float]:
        with self.lock:
            return self.first_completion_monotonic


class MultiPortLogger:
    def __init__(
        self,
        log_dir: Path,
        observer: Optional[Callable[[str, str, str, float], None]] = None,
    ) -> None:
        self.log_dir = log_dir
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.combined_path = self.log_dir / "combined.log"
        self.command_path = self.log_dir / "commands.log"
        self.ports: Dict[str, serial.Serial] = {}
        self.port_files: Dict[str, object] = {}
        self.reader_threads: List[threading.Thread] = []
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.observer = observer

        self.combined_file = self.combined_path.open("w", encoding="utf-8", newline="\n")
        self.command_file = self.command_path.open("w", encoding="utf-8", newline="\n")

    def open_ports(self) -> None:
        for port, cfg in PORT_CONFIG.items():
            ser = serial.Serial(
                port=port,
                baudrate=cfg["baud"],
                timeout=0.2,
                write_timeout=1.0,
            )
            ser.dtr = True
            self.ports[port] = ser
            self.port_files[port] = (self.log_dir / f"{port}.log").open(
                "w",
                encoding="utf-8",
                newline="\n",
            )
            self.mark(f"PORT {port} opened at {cfg['baud']} baud")

    def start_readers(self) -> None:
        for port, ser in self.ports.items():
            thread = threading.Thread(target=self._reader, args=(port, ser), daemon=True)
            thread.start()
            self.reader_threads.append(thread)

    def _reader(self, port: str, ser: serial.Serial) -> None:
        while not self.stop_event.is_set():
            try:
                raw = ser.readline()
            except Exception as exc:  # pragma: no cover - serial runtime
                self.mark(f"READ ERROR {port}: {exc}")
                break

            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            ts = now_iso()
            mono = time.monotonic()
            row = f"{ts} [{port}] {line}"

            with self.lock:
                self.combined_file.write(row + "\n")
                self.combined_file.flush()
                port_file = self.port_files[port]
                port_file.write(f"{ts} {line}\n")
                port_file.flush()

            if self.observer is not None:
                self.observer(port, ts, line, mono)

    def mark(self, text: str) -> None:
        ts = now_iso()
        row = f"{ts} [MARK] {text}"
        with self.lock:
            self.combined_file.write(row + "\n")
            self.combined_file.flush()

    def send(self, port: str, cmd: str) -> None:
        ts = now_iso()
        row = f"{ts} -> {port}: {cmd}"
        with self.lock:
            self.command_file.write(row + "\n")
            self.command_file.flush()
            self.combined_file.write(f"{ts} [CMD ] {port} {cmd}\n")
            self.combined_file.flush()

        payload = (cmd + "\n").encode("utf-8")
        ser = self.ports[port]
        ser.write(payload)
        ser.flush()

    def close(self) -> None:
        self.stop_event.set()
        time.sleep(0.4)
        for ser in self.ports.values():
            try:
                ser.close()
            except Exception:
                pass
        for file_obj in self.port_files.values():
            file_obj.close()
        self.combined_file.close()
        self.command_file.close()


def send_start_snapshot(logger: MultiPortLogger, pause_s: float) -> None:
    logger.mark("START SNAPSHOT")
    for port, cmd in (
        ("COM12", "STATE"),
        ("COM12", "COMMON STATUS"),
        ("COM12", "OTCYCLE STATUS"),
        ("COM12", "SEAL STATUS"),
        ("COM10", "SEAL STATUS"),
        ("COM10", "OTCYCLE STATUS"),
        ("COM11", "I"),
    ):
        logger.send(port, cmd)
        time.sleep(pause_s)


def send_poll_snapshot(logger: MultiPortLogger, poll_index: int, pause_s: float) -> None:
    logger.mark(f"POLL SNAPSHOT #{poll_index:02d}")
    for port, cmd in (
        ("COM12", "COMMON STATUS"),
        ("COM12", "OTCYCLE STATUS"),
        ("COM12", "SEAL STATUS"),
        ("COM10", "SEAL STATUS"),
        ("COM10", "OTCYCLE STATUS"),
    ):
        logger.send(port, cmd)
        time.sleep(pause_s)

    if poll_index % 3 == 0:
        logger.send("COM12", "STATE")
        time.sleep(pause_s)
        logger.send("COM11", "I")
        time.sleep(pause_s)


def load_port_log(path: Path) -> List[LogLine]:
    if not path.exists():
        return []
    rows: List[LogLine] = []
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if len(raw) < 24:
            continue
        ts = raw[:23]
        msg = raw[24:] if len(raw) > 24 else ""
        rows.append(LogLine(ts=ts, dt=parse_ts(ts), text=msg))
    return rows


def load_commands(path: Path) -> List[Tuple[str, Optional[datetime], str, str]]:
    if not path.exists():
        return []
    rows: List[Tuple[str, Optional[datetime], str, str]] = []
    line_re = re.compile(
        r"^(?P<ts>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3}) -> (?P<port>COM\d+): (?P<cmd>.+)$"
    )
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = line_re.match(raw)
        if not match:
            continue
        ts = match.group("ts")
        rows.append((ts, parse_ts(ts), match.group("port"), match.group("cmd")))
    return rows


def first_match(lines: Sequence[LogLine], needles: Sequence[str]) -> Optional[LogLine]:
    for line in lines:
        if any(needle in line.text for needle in needles):
            return line
    return None


def first_match_after(
    lines: Sequence[LogLine],
    needles: Sequence[str],
    after_dt: Optional[datetime],
) -> Optional[LogLine]:
    for line in lines:
        if after_dt is not None and line.dt is not None and line.dt < after_dt:
            continue
        if any(needle in line.text for needle in needles):
            return line
    return None


def count_matches(lines: Sequence[LogLine], needles: Sequence[str]) -> int:
    count = 0
    for line in lines:
        if any(needle in line.text for needle in needles):
            count += 1
    return count


def extract_wait_done_metrics(com10_lines: Sequence[LogLine]) -> Tuple[int, int]:
    entries = 0
    samples = 0
    prev_state: Optional[str] = None
    state_re = re.compile(r"run_state=([a-z_]+)")
    for line in com10_lines:
        match = state_re.search(line.text)
        if not match:
            continue
        state = match.group(1)
        if state == "wait_done":
            samples += 1
            if prev_state != "wait_done":
                entries += 1
        prev_state = state
    return entries, samples


def parse_done_filter_ms(line: Optional[LogLine]) -> Optional[int]:
    if line is None:
        return None
    match = re.search(r"done_filter_ms=(\d+)", line.text)
    if not match:
        return None
    return int(match.group(1))


def find_common_start_cmd_ts(commands: Sequence[Tuple[str, Optional[datetime], str, str]]) -> Optional[str]:
    for ts, _dt, port, cmd in commands:
        if port == "COM12" and cmd.strip().upper() == "COMMON START":
            return ts
    return None


def load_disconnect_events(combined_path: Path) -> List[str]:
    if not combined_path.exists():
        return []
    events: List[str] = []
    for raw in combined_path.read_text(encoding="utf-8", errors="replace").splitlines():
        if "READ ERROR" in raw or "SerialException" in raw:
            events.append(raw)
    return events


def tail_full(lines: Sequence[LogLine], count: int = 10) -> List[str]:
    if not lines:
        return []
    return [line.full() for line in lines[-count:]]


def build_summary(run_dir: Path) -> Dict[str, object]:
    com10_lines = load_port_log(run_dir / "COM10.log")
    com11_lines = load_port_log(run_dir / "COM11.log")
    com12_lines = load_port_log(run_dir / "COM12.log")
    commands = load_commands(run_dir / "commands.log")
    disconnect_events = load_disconnect_events(run_dir / "combined.log")

    run_start = None
    run_end = None
    if com10_lines or com11_lines or com12_lines:
        all_lines = [*com10_lines, *com11_lines, *com12_lines]
        all_lines.sort(key=lambda line: line.dt or datetime.min)
        run_start = all_lines[0].ts
        run_end = all_lines[-1].ts

    first_completion = first_match(
        com10_lines,
        ("SEAL: cycle complete input became active.", "SEAL: cycle complete (synthetic DONE)."),
    )
    first_completion_dt = first_completion.dt if first_completion else None

    common_progress = first_match_after(com12_lines, COMMON_PROGRESS_PATTERNS, first_completion_dt)

    wait_done_entries, wait_done_samples = extract_wait_done_metrics(com10_lines)
    wait_done_heartbeats = count_matches(com10_lines, ("SEAL WAIT_DONE:",))

    valid_completion_count = count_matches(
        com10_lines,
        ("SEAL: cycle complete input became active.", "SEAL: cycle complete (synthetic DONE)."),
    )
    synthetic_completion_count = count_matches(com10_lines, ("SEAL: cycle complete (synthetic DONE).",))

    start_com12_common = first_match(com12_lines, ("COMMON: active=",))
    start_com12_otcycle = first_match(com12_lines, ("OTCYCLE: active=",))
    start_com12_seal = first_match(com12_lines, ("SEAL: start_out=",))
    start_com10_seal = first_match(com10_lines, ("SEAL: run_state=",))
    start_com10_otcycle = first_match(com10_lines, ("OTCYCLE: active=",))

    first_wait_done = first_match(com10_lines, ("SEAL: run_state=wait_done", "SEAL WAIT_DONE:"))
    first_vfdtick_timeout = first_match(com12_lines, ("VFDTICK start timeout",))
    first_otcycle_failed = first_match(com12_lines, ("OTCYCLE failed",))

    summary: Dict[str, object] = {
        "run_dir": str(run_dir).replace("\\", "/"),
        "run_start": run_start,
        "run_end": run_end,
        "metrics": {
            "common_start_cmd_ts": find_common_start_cmd_ts(commands),
            "common_start_seen": first_match(com12_lines, ("COMMON: waiting initial batch", "COMMON TRACE:")) is not None,
            "common_progress_after_done": common_progress is not None,
            "common_progress_line": common_progress.full() if common_progress else None,
            "wait_done_entries": wait_done_entries,
            "wait_done_status_samples": wait_done_samples,
            "wait_done_heartbeats": wait_done_heartbeats,
            "valid_completion_count": valid_completion_count,
            "synthetic_completion_count": synthetic_completion_count,
            "outside_waitdone_count": count_matches(
                com10_lines, ("SEAL: DONE effective rise outside WaitDone",)
            ),
            "done_raw_active_count": count_matches(com10_lines, ("SEAL DONE RAW: active",)),
            "done_raw_released_count": count_matches(com10_lines, ("SEAL DONE RAW: released",)),
            "done_eff_released_count": count_matches(com10_lines, ("SEAL DONE EFF: released.",)),
            "vfdtick_timeout_count": count_matches(com12_lines, ("VFDTICK start timeout",)),
            "otcycle_failed_count": count_matches(com12_lines, ("OTCYCLE failed",)),
            "i2c_recover_count": count_matches(com12_lines, ("I2C RECOVER",)),
            "manip_work_events": count_matches(
                com11_lines, ("Шаг рабочего цикла", "work cycle", "WORK CYCLE")
            ),
        },
        "start": {
            "com12_common": start_com12_common.full() if start_com12_common else None,
            "com12_otcycle": start_com12_otcycle.full() if start_com12_otcycle else None,
            "com12_seal": start_com12_seal.full() if start_com12_seal else None,
            "com10_seal": start_com10_seal.full() if start_com10_seal else None,
            "com10_otcycle": start_com10_otcycle.full() if start_com10_otcycle else None,
            "com10_done_filter_ms": parse_done_filter_ms(start_com10_seal),
            "com11_right": None,
            "com11_zup": None,
            "com11_grip_open": None,
        },
        "events": {
            "wait_done_first_ts": first_wait_done.full() if first_wait_done else None,
            "completion_first_ts": first_completion.full() if first_completion else None,
            "vfdtick_timeout_first": first_vfdtick_timeout.full() if first_vfdtick_timeout else None,
            "otcycle_failed_first": first_otcycle_failed.full() if first_otcycle_failed else None,
        },
        "last_lines": {
            "COM12": tail_full(com12_lines, count=10),
            "COM10": tail_full(com10_lines, count=10),
            "COM11": tail_full(com11_lines, count=10),
        },
        "disconnect_events": disconnect_events,
    }
    return summary


def write_report(run_dir: Path, summary: Dict[str, object]) -> None:
    metrics = summary.get("metrics", {})
    start = summary.get("start", {})
    events = summary.get("events", {})
    last_lines = summary.get("last_lines", {})
    disconnect_events = summary.get("disconnect_events", [])

    lines: List[str] = []
    lines.append("fyl field run report: SEAL DONE wait-done manual button validation")
    lines.append(f"run_start={safe_text(summary.get('run_start'))}")
    lines.append(f"run_end={safe_text(summary.get('run_end'))}")
    lines.append("")

    lines.append("[START STATUS]")
    lines.append(f"COM12 COMMON STATUS: {safe_text(start.get('com12_common'))}")
    lines.append(f"COM12 OTCYCLE STATUS: {safe_text(start.get('com12_otcycle'))}")
    lines.append(f"COM12 SEAL STATUS: {safe_text(start.get('com12_seal'))}")
    lines.append(f"COM10 SEAL STATUS: {safe_text(start.get('com10_seal'))}")
    lines.append(f"COM10 OTCYCLE STATUS: {safe_text(start.get('com10_otcycle'))}")
    lines.append(f"COM10 done_filter_ms: {safe_text(str(start.get('com10_done_filter_ms')) if start.get('com10_done_filter_ms') is not None else None)}")
    lines.append(f"COM11 pose right_pressed: {safe_text(start.get('com11_right'))}")
    lines.append(f"COM11 pose z_up: {safe_text(start.get('com11_zup'))}")
    lines.append(f"COM11 pose grip_open: {safe_text(start.get('com11_grip_open'))}")
    lines.append("")

    lines.append("[RUN METRICS]")
    metric_keys = (
        "common_start_cmd_ts",
        "common_start_seen",
        "wait_done_entries",
        "wait_done_status_samples",
        "wait_done_heartbeats",
        "valid_completion_count",
        "synthetic_completion_count",
        "outside_waitdone_count",
        "done_raw_active_count",
        "done_raw_released_count",
        "done_eff_released_count",
        "manip_work_events",
        "common_progress_after_done",
        "common_progress_line",
        "vfdtick_timeout_count",
        "otcycle_failed_count",
        "i2c_recover_count",
    )
    for key in metric_keys:
        lines.append(f"{key}: {safe_text(str(metrics.get(key)) if metrics.get(key) is not None else None)}")
    lines.append("")

    lines.append("[FIRST EVENTS]")
    lines.append(f"first_wait_done: {safe_text(events.get('wait_done_first_ts'))}")
    lines.append(f"first_completion: {safe_text(events.get('completion_first_ts'))}")
    lines.append(f"first_vfdtick_timeout: {safe_text(events.get('vfdtick_timeout_first'))}")
    lines.append(f"first_otcycle_failed: {safe_text(events.get('otcycle_failed_first'))}")
    lines.append("")

    lines.append("[LAST LINES BEFORE STOP]")
    for port in ("COM12", "COM10", "COM11"):
        lines.append(f"{port}:")
        tail = last_lines.get(port, [])
        if not tail:
            lines.append("  <none>")
            continue
        for row in tail:
            lines.append(f"  {row}")
    lines.append("")

    lines.append("[DISCONNECT EVENTS]")
    if disconnect_events:
        for row in disconnect_events:
            lines.append(row)
    else:
        lines.append("none")

    (run_dir / "report.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def save_summary_json(run_dir: Path, summary: Dict[str, object]) -> None:
    (run_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def run_capture(args: argparse.Namespace) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path("logs") / "field_runs" / f"{stamp}_{args.tag}"

    signals = LiveSignals()
    logger = MultiPortLogger(log_dir=run_dir, observer=signals.observe)

    logger.open_ports()
    logger.start_readers()

    prompted_press = False
    completion_marked = False
    start_monotonic = time.monotonic()

    try:
        logger.mark("BOOT STABILIZE START")
        time.sleep(args.boot_settle_s)

        send_start_snapshot(logger, pause_s=args.cmd_pause_s)

        logger.mark("COMMON START")
        logger.send("COM12", "COMMON START")

        poll_index = 0
        next_poll_at = 0.0

        while True:
            elapsed = time.monotonic() - start_monotonic

            if elapsed >= next_poll_at:
                poll_index += 1
                send_poll_snapshot(logger, poll_index=poll_index, pause_s=args.cmd_pause_s)
                next_poll_at += args.poll_every_s

            if signals.wait_done_event.is_set() and not prompted_press:
                logger.mark("ACTION REQUIRED: press DONE button once, hold 0.6-1.0s, then release.")
                print()
                print("ACTION: WaitDone detected on COM10.")
                print("Press physical DONE button ONCE and hold for 0.6-1.0 seconds, then release.")
                print()
                prompted_press = True

            if signals.completion_event.is_set() and not completion_marked:
                logger.mark("FIRST COMPLETION DETECTED")
                completion_marked = True

            if elapsed >= args.max_seconds:
                logger.mark("STOP: max_seconds reached")
                break

            first_completion_mono = signals.get_first_completion_monotonic()
            if (
                elapsed >= args.min_seconds
                and first_completion_mono is not None
                and (time.monotonic() - first_completion_mono) >= args.post_completion_s
            ):
                logger.mark("STOP: post-completion window captured")
                break

            time.sleep(0.25)

        logger.mark("FINAL SNAPSHOT")
        send_start_snapshot(logger, pause_s=args.cmd_pause_s)
        logger.mark("FIELD RUN COMPLETE")
        return run_dir
    finally:
        logger.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run COM10/COM11/COM12 field capture for SEAL DONE wait-done validation."
    )
    parser.add_argument(
        "--tag",
        default="seal_done_waitdone_manual",
        help="Suffix for logs/field_runs/<timestamp>_<tag> directory.",
    )
    parser.add_argument("--boot-settle-s", type=float, default=8.0, help="Initial wait before first commands.")
    parser.add_argument("--poll-every-s", type=float, default=12.0, help="Periodic status poll interval.")
    parser.add_argument("--cmd-pause-s", type=float, default=0.35, help="Delay between sequential commands.")
    parser.add_argument("--min-seconds", type=float, default=90.0, help="Minimum capture duration.")
    parser.add_argument(
        "--post-completion-s",
        type=float,
        default=45.0,
        help="Keep capturing this long after first completion event.",
    )
    parser.add_argument("--max-seconds", type=float, default=240.0, help="Hard timeout for whole capture.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    run_dir = run_capture(args)
    summary = build_summary(run_dir)
    save_summary_json(run_dir, summary)
    write_report(run_dir, summary)

    print(str(run_dir))
    print(run_dir / "report.txt")
    print(run_dir / "summary.json")


if __name__ == "__main__":
    main()
