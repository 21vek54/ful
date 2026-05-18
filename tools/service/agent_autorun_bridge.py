#!/usr/bin/env python3
"""Strict bridge: parse AUTO_RUN trigger and execute serial_multi_capture safely.

Expected trigger format inside agent response text:

AUTO_RUN:
python tools/service/serial_multi_capture.py --scenario standard ...
"""

from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import List, Optional


TRIGGER_MARKER = "AUTO_RUN:"
RESULT_MARKER = "AUTO_RUN_BRIDGE_RESULT_JSON:"
RESULT_SCHEMA = "fyl.agent_autorun_bridge.v1"
SERIAL_CAPTURE_REL = Path("tools/service/serial_multi_capture.py")
ALLOWED_PYTHON_EXECUTABLES = {"python", "python3", "py"}

EXIT_OK = 0
EXIT_INVALID_TRIGGER = 10
EXIT_RUN_FAILED = 20
EXIT_RUN_TIMEOUT = 21
EXIT_BRIDGE_ERROR = 30


class TriggerError(ValueError):
    pass


@dataclass
class BridgeResult:
    schema: str = RESULT_SCHEMA
    status: str = "no_trigger"
    trigger_found: bool = False
    error: str = ""
    command_line: str = ""
    command_argv: List[str] = field(default_factory=list)
    executed: bool = False
    dry_run: bool = False
    exit_code: Optional[int] = None
    timed_out: bool = False
    duration_s: Optional[float] = None
    stdout_tail: List[str] = field(default_factory=list)
    stderr_tail: List[str] = field(default_factory=list)
    logs_dir: str = ""
    session_trace: str = ""


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Read agent response text, detect strict AUTO_RUN marker, "
            "and run serial_multi_capture.py safely."
        )
    )
    parser.add_argument(
        "--input-file",
        type=Path,
        default=None,
        help="Read agent response text from file. If omitted, read stdin.",
    )
    parser.add_argument(
        "--result-file",
        type=Path,
        default=None,
        help="Optional output file for JSON service result.",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=0.0,
        help="Optional run timeout in seconds (0 = no timeout).",
    )
    parser.add_argument(
        "--max-tail-lines",
        type=int,
        default=80,
        help="How many lines of stdout/stderr to keep in result tails.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Parse/validate trigger but do not run the command.",
    )
    return parser.parse_args()


def read_input_text(input_file: Optional[Path]) -> str:
    if input_file is None:
        return sys.stdin.read()
    return input_file.read_text(encoding="utf-8", errors="replace")


def normalize_marker_line(line: str) -> str:
    return line.lstrip("\ufeff").strip()


def extract_command_line(agent_text: str) -> Optional[str]:
    lines = agent_text.splitlines()
    marker_indices = [idx for idx, line in enumerate(lines) if normalize_marker_line(line) == TRIGGER_MARKER]

    if not marker_indices:
        return None
    if len(marker_indices) > 1:
        raise TriggerError("Multiple AUTO_RUN markers found. Expected exactly one.")

    marker_idx = marker_indices[0]
    if marker_idx + 1 >= len(lines):
        raise TriggerError("AUTO_RUN marker found without command line.")

    command_line = lines[marker_idx + 1].strip()
    if not command_line:
        raise TriggerError("AUTO_RUN command line is empty.")

    tail_non_empty = [line for line in lines[marker_idx + 2 :] if line.strip()]
    if tail_non_empty:
        raise TriggerError(
            "AUTO_RUN block must end after the command line (no extra non-empty lines)."
        )

    return command_line


def normalize_script_path(token: str, base_dir: Path) -> Path:
    token_path = Path(token.strip().strip("\"'"))
    if not token_path.is_absolute():
        token_path = base_dir / token_path
    return token_path.resolve()


def validate_command_line(command_line: str, base_dir: Path) -> List[str]:
    try:
        argv = shlex.split(command_line, posix=False)
    except ValueError as exc:
        raise TriggerError(f"Unable to parse command line: {exc}") from exc

    if len(argv) < 2:
        raise TriggerError("AUTO_RUN command must include python executable and script path.")

    python_exe = argv[0].strip().strip("\"'").lower()
    if python_exe not in ALLOWED_PYTHON_EXECUTABLES:
        raise TriggerError(
            f"First token must be one of: {sorted(ALLOWED_PYTHON_EXECUTABLES)}"
        )

    target_script = normalize_script_path(argv[1], base_dir=base_dir)
    expected_script = (base_dir / SERIAL_CAPTURE_REL).resolve()
    if target_script != expected_script:
        raise TriggerError(
            "Second token must point to tools/service/serial_multi_capture.py"
        )

    normalized_argv = [argv[0], str(expected_script), *argv[2:]]
    return normalized_argv


def tail_lines(text: str, max_lines: int) -> List[str]:
    if max_lines <= 0:
        return []
    lines = text.splitlines()
    if len(lines) <= max_lines:
        return lines
    return lines[-max_lines:]


def extract_value_after_prefix(lines: List[str], prefix: str) -> str:
    for line in reversed(lines):
        if prefix in line:
            return line.split(prefix, 1)[1].strip()
    return ""


def run_capture_command(argv: List[str], base_dir: Path, timeout_s: float) -> subprocess.CompletedProcess:
    timeout = timeout_s if timeout_s and timeout_s > 0 else None
    return subprocess.run(
        argv,
        cwd=str(base_dir),
        text=True,
        capture_output=True,
        encoding="utf-8",
        errors="replace",
        timeout=timeout,
        check=False,
    )


def write_result(result: BridgeResult, result_file: Optional[Path]) -> None:
    payload = json.dumps(asdict(result), ensure_ascii=False, indent=2)
    print(RESULT_MARKER)
    print(payload)
    if result_file is not None:
        result_file.write_text(payload + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    result = BridgeResult()
    base_dir = repo_root()

    try:
        agent_text = read_input_text(args.input_file)
        lines = agent_text.splitlines()
        result.trigger_found = any(normalize_marker_line(line) == TRIGGER_MARKER for line in lines)
        command_line = extract_command_line(agent_text)
        if command_line is None:
            result.status = "no_trigger"
            write_result(result, args.result_file)
            return EXIT_OK

        result.command_line = command_line
        result.command_argv = validate_command_line(command_line, base_dir=base_dir)

        if args.dry_run:
            result.status = "dry_run_ready"
            result.dry_run = True
            write_result(result, args.result_file)
            return EXIT_OK

        started = time.monotonic()
        try:
            completed = run_capture_command(result.command_argv, base_dir=base_dir, timeout_s=args.timeout_s)
        except subprocess.TimeoutExpired as exc:
            result.status = "run_timeout"
            result.executed = True
            result.timed_out = True
            result.exit_code = None
            result.duration_s = round(time.monotonic() - started, 3)
            result.stdout_tail = tail_lines(exc.stdout or "", args.max_tail_lines)
            result.stderr_tail = tail_lines(exc.stderr or "", args.max_tail_lines)
            write_result(result, args.result_file)
            return EXIT_RUN_TIMEOUT

        result.executed = True
        result.exit_code = completed.returncode
        result.duration_s = round(time.monotonic() - started, 3)
        result.stdout_tail = tail_lines(completed.stdout or "", args.max_tail_lines)
        result.stderr_tail = tail_lines(completed.stderr or "", args.max_tail_lines)
        result.logs_dir = extract_value_after_prefix(result.stdout_tail, "Логи сохранены в:")
        result.session_trace = extract_value_after_prefix(result.stdout_tail, "Session trace сохранен:")

        if completed.returncode == 0:
            result.status = "run_completed"
            write_result(result, args.result_file)
            return EXIT_OK

        result.status = "run_failed"
        write_result(result, args.result_file)
        return EXIT_RUN_FAILED

    except TriggerError as exc:
        result.status = "invalid_trigger"
        result.error = str(exc)
        write_result(result, args.result_file)
        return EXIT_INVALID_TRIGGER
    except Exception as exc:  # pragma: no cover - guard rail for external runtime
        result.status = "bridge_error"
        result.error = str(exc)
        write_result(result, args.result_file)
        return EXIT_BRIDGE_ERROR


if __name__ == "__main__":
    raise SystemExit(main())
