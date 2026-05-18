#!/usr/bin/env python3
"""Full bench repair loop: patch -> flash -> run -> analyze until done.

This loop runs outside chat via OpenAI Responses API and enforces a strict
action protocol so the model can iterate safely in environment "ful".
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import textwrap
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


DEFAULT_MODEL = "gpt-5"
DEFAULT_API_BASE = "https://api.openai.com/v1"
DEFAULT_MAX_STEPS = 12
DEFAULT_TIMEOUT_S = 600.0
DEFAULT_LOG_ROOT = Path("logs") / "agent_ful_repair_loop"
DEFAULT_API_KEY_FILE_REL = Path("tools/service/openai_api_key_local.txt")
DEFAULT_MAX_TAIL_LINES = 180
DEFAULT_MAX_TRACE_TAIL_LINES = 220
DEFAULT_MAX_REPEAT_FAILS = 3
DEFAULT_RUN_TIMEOUT_S = 420.0

STEP_RESULT_MARKER = "AUTO_FUL_STEP_RESULT_JSON:"
SERIAL_CAPTURE_REL = Path("tools/service/serial_multi_capture.py")
ALLOWED_PYTHON_EXECUTABLES = {"python", "python3", "py"}

ACTION_APPLY_DIFF = "AUTO_APPLY_DIFF:"
ACTION_FLASH = "AUTO_FLASH:"
ACTION_RUN = "AUTO_RUN:"
ACTION_DONE = "AUTO_DONE:"
ALLOWED_MARKERS = [ACTION_APPLY_DIFF, ACTION_FLASH, ACTION_RUN, ACTION_DONE]

ALLOWED_DIFF_PREFIXES = (
    "firmware/",
    "archive/firmware/",
    "tools/service/",
    "docs/",
    "Plan/",
    "bin/",
)

FLASH_TARGETS = {"com10", "com11", "com12", "all"}
FLASH_TARGET_TO_PROJECT = {
    "com10": Path("firmware/conveyor_control_com10"),
    "com11": Path("firmware/manipulator_com11"),
    "com12": Path("archive/firmware/master_control_com12_legacy_2026-03-25"),
}

DEFAULT_PROTOCOL_INSTRUCTIONS = textwrap.dedent(
    """
    You are in strict FULL repair loop for environment ful.
    Objective: restore working production flow on stand.

    Output protocol (strict):
    - Return exactly one action block per turn.
    - No non-empty lines before marker.
    - Supported markers:
      1) AUTO_APPLY_DIFF:
         ```diff
         <unified git diff>
         ```
      2) AUTO_FLASH:
         <com10|com11|com12|all>
      3) AUTO_RUN:
         python tools/service/serial_multi_capture.py <args>
      4) AUTO_DONE:
         <single-line summary when objective is reached>
    - Do not emit any other markers.
    - Prefer minimal safe patches and verify with flash+run.
    - If previous step failed, adapt and continue iterating.
    - AUTO_RUN safety: use scenario mode and explicit auto-stop limiter.
    """
).strip()


class ProtocolError(ValueError):
    pass


@dataclass
class ParsedAction:
    marker: str
    payload: str


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def now_stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run strict ful repair loop: apply diff, flash, run, and iterate."
    )
    parser.add_argument("--model", default=DEFAULT_MODEL, help=f"Model name (default: {DEFAULT_MODEL}).")
    parser.add_argument("--goal", default="", help="Goal text. If empty, use --goal-file or stdin.")
    parser.add_argument("--goal-file", type=Path, default=None, help="Goal text file.")
    parser.add_argument(
        "--instructions-file",
        type=Path,
        default=None,
        help="Optional custom protocol instructions.",
    )
    parser.add_argument("--max-steps", type=int, default=DEFAULT_MAX_STEPS, help="Max loop steps.")
    parser.add_argument(
        "--max-repeat-fails",
        type=int,
        default=DEFAULT_MAX_REPEAT_FAILS,
        help="Stop if the same failure signature repeats N times.",
    )
    parser.add_argument(
        "--api-base",
        default=DEFAULT_API_BASE,
        help=f"OpenAI API base URL (default: {DEFAULT_API_BASE}).",
    )
    parser.add_argument(
        "--reasoning-effort",
        choices=("low", "medium", "high", "xhigh"),
        default="",
        help="Optional reasoning.effort.",
    )
    parser.add_argument(
        "--api-key-file",
        type=Path,
        default=None,
        help=(
            "Path to local key file. If omitted, uses "
            "tools/service/openai_api_key_local.txt then OPENAI_API_KEY env."
        ),
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help=f"HTTP timeout in seconds (default: {DEFAULT_TIMEOUT_S}).",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=None,
        help="Session dir. Default: logs/agent_ful_repair_loop/<timestamp>.",
    )
    parser.add_argument(
        "--max-tail-lines",
        type=int,
        default=DEFAULT_MAX_TAIL_LINES,
        help="Tail lines for subprocess stdout/stderr in step results.",
    )
    parser.add_argument(
        "--max-trace-tail-lines",
        type=int,
        default=DEFAULT_MAX_TRACE_TAIL_LINES,
        help="Tail lines loaded from session trace after AUTO_RUN.",
    )
    parser.add_argument(
        "--run-timeout-s",
        type=float,
        default=DEFAULT_RUN_TIMEOUT_S,
        help=f"Hard timeout for AUTO_RUN subprocess in seconds (default: {DEFAULT_RUN_TIMEOUT_S}).",
    )
    return parser.parse_args()


def read_text_file(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def read_goal(args: argparse.Namespace) -> str:
    if args.goal_file is not None:
        return read_text_file(args.goal_file).strip()
    if args.goal.strip():
        return args.goal.strip()
    stdin_text = sys.stdin.read().strip()
    if stdin_text:
        return stdin_text
    raise ValueError("Goal is empty. Provide --goal, --goal-file, or stdin text.")


def read_instructions(args: argparse.Namespace) -> str:
    if args.instructions_file is None:
        return DEFAULT_PROTOCOL_INSTRUCTIONS
    text = read_text_file(args.instructions_file).strip()
    return text if text else DEFAULT_PROTOCOL_INSTRUCTIONS


def make_session_dir(args: argparse.Namespace, base_dir: Path) -> Path:
    if args.log_dir is not None:
        session_dir = args.log_dir
    else:
        session_dir = base_dir / DEFAULT_LOG_ROOT / now_stamp()
    session_dir.mkdir(parents=True, exist_ok=True)
    return session_dir


def normalize_key_candidate(text: str) -> str:
    token = text.strip().lstrip("\ufeff")
    if not token:
        return ""
    if token.startswith("OPENAI_API_KEY="):
        token = token.split("=", 1)[1].strip()
    if token and token[0] in {"'", '"'} and token[-1:] == token[0]:
        token = token[1:-1].strip()
    return token


def key_file_path(args: argparse.Namespace, base_dir: Path) -> Path:
    if args.api_key_file is not None:
        path = args.api_key_file
        if not path.is_absolute():
            path = base_dir / path
        return path.resolve()
    return (base_dir / DEFAULT_API_KEY_FILE_REL).resolve()


def read_api_key_from_file(path: Path) -> str:
    raw = path.read_text(encoding="utf-8", errors="replace")
    for line in raw.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        candidate = normalize_key_candidate(stripped)
        if candidate:
            return candidate
    return ""


def resolve_api_key(args: argparse.Namespace, base_dir: Path) -> Tuple[str, str]:
    local_path = key_file_path(args, base_dir=base_dir)
    if local_path.exists():
        key = read_api_key_from_file(local_path)
        if key:
            return key, str(local_path)
        raise ValueError(f"API key file exists but no key found: {local_path}")

    env_key = normalize_key_candidate(os.environ.get("OPENAI_API_KEY", ""))
    if env_key:
        return env_key, "env:OPENAI_API_KEY"
    raise ValueError(
        "OpenAI API key not found. Put key into tools/service/openai_api_key_local.txt "
        "or set OPENAI_API_KEY."
    )


def post_responses_create(
    *,
    api_key: str,
    api_base: str,
    model: str,
    instructions: str,
    user_text: str,
    previous_response_id: Optional[str],
    reasoning_effort: str,
    timeout_s: float,
) -> Dict[str, Any]:
    url = api_base.rstrip("/") + "/responses"
    payload: Dict[str, Any] = {
        "model": model,
        "instructions": instructions,
        "input": [{"role": "user", "content": [{"type": "input_text", "text": user_text}]}],
    }
    if previous_response_id:
        payload["previous_response_id"] = previous_response_id
    if reasoning_effort:
        payload["reasoning"] = {"effort": reasoning_effort}

    request = Request(
        url=url,
        data=json.dumps(payload, ensure_ascii=False).encode("utf-8"),
        headers={"Authorization": f"Bearer {api_key}", "Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urlopen(request, timeout=timeout_s) as resp:
            return json.loads(resp.read().decode("utf-8", errors="replace"))
    except HTTPError as exc:
        err = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"HTTP {exc.code}: {err}") from exc
    except URLError as exc:
        raise RuntimeError(f"Network error: {exc}") from exc


def _text_from_content_item(item: Dict[str, Any]) -> str:
    if not isinstance(item, dict):
        return ""
    if isinstance(item.get("text"), str):
        return item["text"]
    text_obj = item.get("text")
    if isinstance(text_obj, dict) and isinstance(text_obj.get("value"), str):
        return text_obj["value"]
    if isinstance(item.get("value"), str):
        return item["value"]
    return ""


def extract_assistant_text(response_json: Dict[str, Any]) -> str:
    top = response_json.get("output_text")
    if isinstance(top, str) and top.strip():
        return top.strip()
    chunks: List[str] = []
    for out_item in response_json.get("output", []):
        if not isinstance(out_item, dict):
            continue
        for content_item in out_item.get("content", []):
            text = _text_from_content_item(content_item).strip()
            if text:
                chunks.append(text)
    return "\n".join(chunks).strip()


def normalize_marker_line(line: str) -> str:
    return line.lstrip("\ufeff").strip()


def parse_single_action(assistant_text: str) -> ParsedAction:
    lines = assistant_text.splitlines()
    marker_hits: List[Tuple[int, str]] = []
    for idx, line in enumerate(lines):
        normalized = normalize_marker_line(line)
        if normalized in ALLOWED_MARKERS:
            marker_hits.append((idx, normalized))

    if not marker_hits:
        raise ProtocolError("No action marker found.")
    if len(marker_hits) > 1:
        raise ProtocolError("Multiple action markers in one response.")

    marker_idx, marker = marker_hits[0]
    if any(line.strip() for line in lines[:marker_idx]):
        raise ProtocolError("Non-empty text before action marker is not allowed.")

    if marker in (ACTION_FLASH, ACTION_RUN, ACTION_DONE):
        if marker_idx + 1 >= len(lines):
            raise ProtocolError(f"{marker} requires payload line.")
        payload = lines[marker_idx + 1].strip()
        if not payload:
            raise ProtocolError(f"{marker} payload is empty.")
        if any(line.strip() for line in lines[marker_idx + 2 :]):
            raise ProtocolError(f"{marker} must not contain extra non-empty lines.")
        return ParsedAction(marker=marker, payload=payload)

    if marker == ACTION_APPLY_DIFF:
        start = marker_idx + 1
        if start >= len(lines):
            raise ProtocolError("AUTO_APPLY_DIFF requires fenced diff block.")
        fence = lines[start].strip()
        if fence not in ("```diff", "```patch", "```"):
            raise ProtocolError("AUTO_APPLY_DIFF must start with fenced diff block.")
        end_idx = -1
        for idx in range(start + 1, len(lines)):
            if lines[idx].strip() == "```":
                end_idx = idx
                break
        if end_idx < 0:
            raise ProtocolError("AUTO_APPLY_DIFF fenced block is not closed.")
        if any(line.strip() for line in lines[end_idx + 1 :]):
            raise ProtocolError("AUTO_APPLY_DIFF must not contain extra non-empty lines.")
        diff_text = "\n".join(lines[start + 1 : end_idx]).strip()
        if not diff_text:
            raise ProtocolError("AUTO_APPLY_DIFF diff is empty.")
        return ParsedAction(marker=marker, payload=diff_text)

    raise ProtocolError(f"Unsupported marker: {marker}")


def tail_lines(text: str, max_lines: int) -> List[str]:
    if max_lines <= 0:
        return []
    rows = text.splitlines()
    return rows[-max_lines:] if len(rows) > max_lines else rows


def parse_diff_paths(diff_text: str) -> List[str]:
    paths: List[str] = []
    for line in diff_text.splitlines():
        if not line.startswith("diff --git "):
            continue
        parts = line.split()
        if len(parts) < 4:
            continue
        b_part = parts[3]
        if not b_part.startswith("b/"):
            continue
        rel = b_part[2:]
        paths.append(rel)
    return paths


def is_safe_repo_rel_path(rel_path: str) -> bool:
    if not rel_path:
        return False
    p = Path(rel_path)
    if p.is_absolute():
        return False
    if ".." in p.parts:
        return False
    normalized = rel_path.replace("\\", "/")
    return normalized.startswith(ALLOWED_DIFF_PREFIXES)


def run_subprocess(
    argv: List[str],
    *,
    cwd: Path,
    timeout_s: Optional[float] = None,
) -> subprocess.CompletedProcess:
    timeout = timeout_s if timeout_s and timeout_s > 0 else None
    return subprocess.run(
        argv,
        cwd=str(cwd),
        text=True,
        capture_output=True,
        encoding="utf-8",
        errors="replace",
        timeout=timeout,
        check=False,
    )


def execute_apply_diff(
    *,
    diff_text: str,
    base_dir: Path,
    session_dir: Path,
    step_index: int,
    max_tail_lines: int,
) -> Dict[str, Any]:
    touched = parse_diff_paths(diff_text)
    if not touched:
        return {
            "ok": False,
            "error": "Diff has no 'diff --git' paths.",
        }
    bad_paths = [path for path in touched if not is_safe_repo_rel_path(path)]
    if bad_paths:
        return {
            "ok": False,
            "error": f"Diff touches forbidden paths: {bad_paths}",
            "touched_paths": touched,
        }

    patch_path = session_dir / f"step_{step_index:02d}.diff"
    patch_path.write_text(diff_text + "\n", encoding="utf-8")

    started = time.monotonic()
    completed = run_subprocess(
        ["git", "apply", "--whitespace=nowarn", "--recount", str(patch_path.resolve())],
        cwd=base_dir,
    )
    duration = round(time.monotonic() - started, 3)

    changed_files: List[str] = []
    if completed.returncode == 0:
        diff_names = run_subprocess(["git", "diff", "--name-only"], cwd=base_dir)
        changed_files = tail_lines(diff_names.stdout or "", 300)

    return {
        "ok": completed.returncode == 0,
        "exit_code": completed.returncode,
        "duration_s": duration,
        "touched_paths": touched,
        "changed_files_tail": changed_files,
        "stdout_tail": tail_lines(completed.stdout or "", max_tail_lines),
        "stderr_tail": tail_lines(completed.stderr or "", max_tail_lines),
        "patch_file": str(patch_path.resolve()),
    }


def execute_flash(
    *,
    target: str,
    base_dir: Path,
    max_tail_lines: int,
) -> Dict[str, Any]:
    token = target.strip().lower()
    if token not in FLASH_TARGETS:
        return {"ok": False, "error": f"Unsupported flash target: {target}"}

    target_list = ["com10", "com11", "com12"] if token == "all" else [token]
    records: List[Dict[str, Any]] = []
    all_ok = True
    for name in target_list:
        project_dir = (base_dir / FLASH_TARGET_TO_PROJECT[name]).resolve()
        started = time.monotonic()
        completed = run_subprocess(["pio", "run", "-t", "upload"], cwd=project_dir)
        duration = round(time.monotonic() - started, 3)
        rec = {
            "target": name,
            "project_dir": str(project_dir),
            "ok": completed.returncode == 0,
            "exit_code": completed.returncode,
            "duration_s": duration,
            "stdout_tail": tail_lines(completed.stdout or "", max_tail_lines),
            "stderr_tail": tail_lines(completed.stderr or "", max_tail_lines),
        }
        records.append(rec)
        if completed.returncode != 0:
            all_ok = False
            break
    return {"ok": all_ok, "target": token, "records": records}


def normalize_script_path(token: str, base_dir: Path) -> Path:
    token_path = Path(token.strip().strip("\"'"))
    if not token_path.is_absolute():
        token_path = base_dir / token_path
    return token_path.resolve()


def validate_run_command(command_line: str, base_dir: Path) -> List[str]:
    try:
        argv = shlex.split(command_line, posix=False)
    except ValueError as exc:
        raise ProtocolError(f"Unable to parse AUTO_RUN command: {exc}") from exc

    if len(argv) < 2:
        raise ProtocolError("AUTO_RUN requires python executable and script path.")
    py_token = argv[0].strip().strip("\"'").lower()
    if py_token not in ALLOWED_PYTHON_EXECUTABLES:
        raise ProtocolError(f"AUTO_RUN first token must be one of: {sorted(ALLOWED_PYTHON_EXECUTABLES)}")

    target_script = normalize_script_path(argv[1], base_dir=base_dir)
    expected = (base_dir / SERIAL_CAPTURE_REL).resolve()
    if target_script != expected:
        raise ProtocolError("AUTO_RUN script must be tools/service/serial_multi_capture.py")

    run_args = argv[2:]
    if "--scenario" not in run_args:
        raise ProtocolError("AUTO_RUN must include --scenario (scenario mode required).")

    safe_limiter_options = {"--duration-s", "--auto-stop-after-s"}
    if not any(opt in run_args for opt in safe_limiter_options):
        raise ProtocolError("AUTO_RUN must include explicit stop limiter: --duration-s or --auto-stop-after-s.")

    return [argv[0], str(expected), *argv[2:]]


def extract_value_after_prefix(lines: List[str], prefix: str) -> str:
    for line in reversed(lines):
        if prefix in line:
            return line.split(prefix, 1)[1].strip()
    return ""


def safe_read_tail(path_text: str, max_lines: int) -> List[str]:
    if not path_text:
        return []
    path = Path(path_text)
    if not path.exists() or not path.is_file():
        return []
    return tail_lines(path.read_text(encoding="utf-8", errors="replace"), max_lines)


def execute_run(
    *,
    command_line: str,
    base_dir: Path,
    max_tail_lines: int,
    max_trace_tail_lines: int,
    run_timeout_s: float,
) -> Dict[str, Any]:
    argv = validate_run_command(command_line, base_dir=base_dir)
    started = time.monotonic()
    try:
        completed = run_subprocess(argv, cwd=base_dir, timeout_s=run_timeout_s)
    except subprocess.TimeoutExpired as exc:
        duration = round(time.monotonic() - started, 3)
        stdout_tail = tail_lines((exc.stdout or ""), max_tail_lines)
        stderr_tail = tail_lines((exc.stderr or ""), max_tail_lines)
        return {
            "ok": False,
            "timed_out": True,
            "error": f"AUTO_RUN timed out after {run_timeout_s:.1f}s.",
            "duration_s": duration,
            "command_line": command_line,
            "command_argv": argv,
            "logs_dir": "",
            "session_trace": "",
            "stdout_tail": stdout_tail,
            "stderr_tail": stderr_tail,
            "session_trace_tail": [],
        }

    duration = round(time.monotonic() - started, 3)
    stdout_tail = tail_lines(completed.stdout or "", max_tail_lines)
    stderr_tail = tail_lines(completed.stderr or "", max_tail_lines)

    logs_dir = extract_value_after_prefix(stdout_tail, "Логи сохранены в:")
    session_trace = extract_value_after_prefix(stdout_tail, "Session trace сохранен:")
    trace_tail = safe_read_tail(session_trace, max_trace_tail_lines)

    return {
        "ok": completed.returncode == 0,
        "exit_code": completed.returncode,
        "duration_s": duration,
        "command_line": command_line,
        "command_argv": argv,
        "logs_dir": logs_dir,
        "session_trace": session_trace,
        "stdout_tail": stdout_tail,
        "stderr_tail": stderr_tail,
        "session_trace_tail": trace_tail,
    }


def build_step_result_payload(step_result: Dict[str, Any]) -> str:
    return STEP_RESULT_MARKER + "\n" + json.dumps(step_result, ensure_ascii=False, indent=2)


def failure_signature(step_result: Dict[str, Any]) -> str:
    action = step_result.get("action", "unknown")
    if step_result.get("ok"):
        return "ok"
    err = str(step_result.get("error", "")).strip()
    exit_code = str(step_result.get("exit_code", ""))
    return f"{action}|{exit_code}|{err[:120]}"


def main() -> int:
    args = parse_args()
    base_dir = repo_root()
    session_dir = make_session_dir(args, base_dir=base_dir)

    try:
        api_key, api_key_source = resolve_api_key(args, base_dir=base_dir)
    except ValueError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2

    try:
        goal_text = read_goal(args)
    except ValueError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2

    instructions = read_instructions(args)
    session_meta = {
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "model": args.model,
        "max_steps": args.max_steps,
        "max_repeat_fails": args.max_repeat_fails,
        "api_base": args.api_base,
        "reasoning_effort": args.reasoning_effort,
        "api_key_source": api_key_source,
        "session_dir": str(session_dir.resolve()),
    }
    (session_dir / "session_meta.json").write_text(
        json.dumps(session_meta, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (session_dir / "instructions.txt").write_text(instructions + "\n", encoding="utf-8")
    (session_dir / "goal.txt").write_text(goal_text + "\n", encoding="utf-8")

    previous_response_id: Optional[str] = None
    user_message = (
        "Environment: ful (confirmed).\n"
        "Iterate until flow is restored or a hard blocker is proven.\n"
        "Goal:\n"
        f"{goal_text}\n"
        "Start now with one strict action block."
    )

    seen_failures: Dict[str, int] = {}

    for step_index in range(1, args.max_steps + 1):
        print("")
        print(f"========== STEP {step_index} ==========")
        print(f"[INFO] Sending message to model {args.model}...")

        try:
            response_json = post_responses_create(
                api_key=api_key,
                api_base=args.api_base,
                model=args.model,
                instructions=instructions,
                user_text=user_message,
                previous_response_id=previous_response_id,
                reasoning_effort=args.reasoning_effort,
                timeout_s=args.timeout_s,
            )
        except RuntimeError as exc:
            print(f"[ERROR] OpenAI request failed: {exc}", file=sys.stderr)
            return 3

        response_file = session_dir / f"step_{step_index:02d}_response.json"
        response_file.write_text(
            json.dumps(response_json, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

        previous_response_id = str(response_json.get("id", "")).strip() or previous_response_id
        assistant_text = extract_assistant_text(response_json)
        if not assistant_text:
            print("[ERROR] Assistant text is empty.", file=sys.stderr)
            return 4

        assistant_file = session_dir / f"step_{step_index:02d}_assistant.txt"
        assistant_file.write_text(assistant_text + "\n", encoding="utf-8")
        print("[ASSISTANT]")
        print(assistant_text)

        step_result: Dict[str, Any] = {
            "schema": "fyl.agent_ful_repair_loop.step_result.v1",
            "step_index": step_index,
            "timestamp": datetime.now().isoformat(timespec="seconds"),
        }

        try:
            parsed = parse_single_action(assistant_text)
            step_result["action"] = parsed.marker
        except ProtocolError as exc:
            step_result["action"] = "PROTOCOL_ERROR"
            step_result["ok"] = False
            step_result["error"] = str(exc)
            result_file = session_dir / f"step_{step_index:02d}_result.json"
            result_file.write_text(json.dumps(step_result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print(f"[STEP] protocol_error: {exc}")
            user_message = build_step_result_payload(step_result)
            continue

        if parsed.marker == ACTION_APPLY_DIFF:
            action_result = execute_apply_diff(
                diff_text=parsed.payload,
                base_dir=base_dir,
                session_dir=session_dir,
                step_index=step_index,
                max_tail_lines=args.max_tail_lines,
            )
            step_result.update(action_result)
        elif parsed.marker == ACTION_FLASH:
            action_result = execute_flash(
                target=parsed.payload,
                base_dir=base_dir,
                max_tail_lines=args.max_tail_lines,
            )
            step_result.update(action_result)
        elif parsed.marker == ACTION_RUN:
            try:
                action_result = execute_run(
                    command_line=parsed.payload,
                    base_dir=base_dir,
                    max_tail_lines=args.max_tail_lines,
                    max_trace_tail_lines=args.max_trace_tail_lines,
                    run_timeout_s=args.run_timeout_s,
                )
            except ProtocolError as exc:
                action_result = {"ok": False, "error": str(exc)}
            step_result.update(action_result)
        elif parsed.marker == ACTION_DONE:
            step_result["ok"] = True
            step_result["summary"] = parsed.payload
            result_file = session_dir / f"step_{step_index:02d}_result.json"
            result_file.write_text(json.dumps(step_result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print("[DONE] Model reported completion.")
            print(f"[DONE] Summary: {parsed.payload}")
            print(f"[DONE] Session logs: {session_dir.resolve()}")
            return 0
        else:
            step_result["ok"] = False
            step_result["error"] = f"Unsupported parsed action: {parsed.marker}"

        result_file = session_dir / f"step_{step_index:02d}_result.json"
        result_file.write_text(json.dumps(step_result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

        print(f"[STEP] action={step_result.get('action')} ok={step_result.get('ok')}")
        if not step_result.get("ok"):
            sig = failure_signature(step_result)
            count = seen_failures.get(sig, 0) + 1
            seen_failures[sig] = count
            print(f"[STEP] failure_signature={sig} repeat={count}")
            if count >= max(1, int(args.max_repeat_fails)):
                print("[STOP] Same failure repeated too many times.")
                print(f"[STOP] Session logs: {session_dir.resolve()}")
                return 5

        user_message = build_step_result_payload(step_result)


    print("")
    print(f"[DONE] Max steps reached ({args.max_steps}).")
    print(f"[DONE] Session logs: {session_dir.resolve()}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("")
        print("[STOP] Interrupted by user (Ctrl+C).")
        raise SystemExit(130)
