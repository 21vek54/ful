#!/usr/bin/env python3
"""Automatic OpenAI loop for bench runs via AUTO_RUN bridge.

Flow:
1) send user prompt to Responses API;
2) save assistant text;
3) run tools/service/agent_autorun_bridge.py on assistant text;
4) send AUTO_RUN_BRIDGE_RESULT_JSON back as next user turn;
5) repeat until bridge says "no_trigger" or max turns reached.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import textwrap
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


DEFAULT_MODEL = "gpt-5"
DEFAULT_API_BASE = "https://api.openai.com/v1"
DEFAULT_MAX_TURNS = 8
DEFAULT_TIMEOUT_S = 600.0
DEFAULT_LOG_ROOT = Path("logs") / "agent_autorun_loop"
DEFAULT_API_KEY_FILE_REL = Path("tools/service/openai_api_key_local.txt")

RESULT_MARKER = "AUTO_RUN_BRIDGE_RESULT_JSON:"
BRIDGE_REL = Path("tools/service/agent_autorun_bridge.py")

DEFAULT_AUTORUN_INSTRUCTIONS = textwrap.dedent(
    """
    You are running in a strict bench auto-run loop.
    If and only if you want a bench run, emit exactly one trigger block:
    AUTO_RUN:
    python tools/service/serial_multi_capture.py <args>

    Strict output contract:
    - The marker line is exactly: AUTO_RUN:
    - The very next non-empty line is exactly one command line.
    - After that command line there must be no non-empty lines.
    - Allowed command is only python ... tools/service/serial_multi_capture.py
    - If no run is needed, do not emit AUTO_RUN at all.
    """
).strip()


@dataclass
class BridgeInvocation:
    result: Dict[str, Any]
    exit_code: int
    result_file: Path
    stdout_file: Path
    stderr_file: Path


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def now_stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run external OpenAI loop with AUTO_RUN bridge automation."
    )
    parser.add_argument("--model", default=DEFAULT_MODEL, help=f"Model name (default: {DEFAULT_MODEL}).")
    parser.add_argument("--max-turns", type=int, default=DEFAULT_MAX_TURNS, help="Max assistant turns.")
    parser.add_argument(
        "--prompt",
        default="",
        help="Initial user prompt text. If omitted, use --prompt-file or stdin.",
    )
    parser.add_argument(
        "--prompt-file",
        type=Path,
        default=None,
        help="Initial user prompt from file.",
    )
    parser.add_argument(
        "--instructions-file",
        type=Path,
        default=None,
        help="Optional custom instructions for model.",
    )
    parser.add_argument(
        "--reasoning-effort",
        choices=("low", "medium", "high", "xhigh"),
        default="",
        help="Optional reasoning.effort for models that support it.",
    )
    parser.add_argument(
        "--api-base",
        default=DEFAULT_API_BASE,
        help=f"OpenAI API base URL (default: {DEFAULT_API_BASE}).",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=DEFAULT_TIMEOUT_S,
        help=f"HTTP timeout in seconds (default: {DEFAULT_TIMEOUT_S}).",
    )
    parser.add_argument(
        "--bridge-timeout-s",
        type=float,
        default=0.0,
        help="Optional timeout passed to bridge (--timeout-s).",
    )
    parser.add_argument(
        "--api-key-file",
        type=Path,
        default=None,
        help=(
            "Path to local API key file. If omitted, tries "
            "tools/service/openai_api_key_local.txt, then OPENAI_API_KEY env."
        ),
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=None,
        help="Session log directory. Default: logs/agent_autorun_loop/<timestamp>",
    )
    return parser.parse_args()


def read_text_file(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def read_initial_prompt(args: argparse.Namespace) -> str:
    if args.prompt_file is not None:
        return read_text_file(args.prompt_file).strip()
    if args.prompt.strip():
        return args.prompt.strip()
    stdin_text = sys.stdin.read().strip()
    if stdin_text:
        return stdin_text
    raise ValueError("Initial prompt is empty. Provide --prompt, --prompt-file, or stdin text.")


def read_instructions(args: argparse.Namespace) -> str:
    if args.instructions_file is None:
        return DEFAULT_AUTORUN_INSTRUCTIONS
    custom = read_text_file(args.instructions_file).strip()
    if not custom:
        return DEFAULT_AUTORUN_INSTRUCTIONS
    return custom


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
    raw_text = path.read_text(encoding="utf-8", errors="replace")
    for line in raw_text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        candidate = normalize_key_candidate(stripped)
        if candidate:
            return candidate
    return ""


def resolve_api_key(args: argparse.Namespace, base_dir: Path) -> Tuple[str, str]:
    local_key_file = key_file_path(args, base_dir=base_dir)
    if local_key_file.exists():
        file_key = read_api_key_from_file(local_key_file)
        if file_key:
            return file_key, str(local_key_file)
        raise ValueError(f"API key file exists but no key found: {local_key_file}")

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
        "input": [
            {
                "role": "user",
                "content": [{"type": "input_text", "text": user_text}],
            }
        ],
    }
    if previous_response_id:
        payload["previous_response_id"] = previous_response_id
    if reasoning_effort:
        payload["reasoning"] = {"effort": reasoning_effort}

    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    headers = {
        "Authorization": f"Bearer {api_key}",
        "Content-Type": "application/json",
    }

    request = Request(url=url, data=body, headers=headers, method="POST")
    try:
        with urlopen(request, timeout=timeout_s) as response:
            raw = response.read().decode("utf-8", errors="replace")
            return json.loads(raw)
    except HTTPError as exc:
        err_text = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"HTTP {exc.code}: {err_text}") from exc
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
    top_text = response_json.get("output_text")
    if isinstance(top_text, str) and top_text.strip():
        return top_text.strip()

    chunks: List[str] = []
    for output_item in response_json.get("output", []):
        if not isinstance(output_item, dict):
            continue
        for content_item in output_item.get("content", []):
            chunk = _text_from_content_item(content_item).strip()
            if chunk:
                chunks.append(chunk)

    if chunks:
        return "\n".join(chunks).strip()
    return ""


def run_bridge(
    *,
    base_dir: Path,
    session_dir: Path,
    turn_index: int,
    assistant_text: str,
    bridge_timeout_s: float,
) -> BridgeInvocation:
    assistant_file = session_dir / f"turn_{turn_index:02d}_assistant.txt"
    result_file = session_dir / f"turn_{turn_index:02d}_bridge_result.json"
    stdout_file = session_dir / f"turn_{turn_index:02d}_bridge_stdout.txt"
    stderr_file = session_dir / f"turn_{turn_index:02d}_bridge_stderr.txt"

    assistant_file.write_text(assistant_text + "\n", encoding="utf-8")

    cmd = [
        sys.executable,
        str((base_dir / BRIDGE_REL).resolve()),
        "--input-file",
        str(assistant_file.resolve()),
        "--result-file",
        str(result_file.resolve()),
    ]
    if bridge_timeout_s > 0:
        cmd.extend(["--timeout-s", str(bridge_timeout_s)])

    completed = subprocess.run(
        cmd,
        cwd=str(base_dir),
        text=True,
        capture_output=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )

    stdout_file.write_text(completed.stdout or "", encoding="utf-8")
    stderr_file.write_text(completed.stderr or "", encoding="utf-8")

    if result_file.exists():
        result = json.loads(result_file.read_text(encoding="utf-8", errors="replace"))
    else:
        result = {
            "schema": "fyl.agent_autorun_bridge.v1",
            "status": "bridge_error",
            "error": "Bridge result file not created.",
            "trigger_found": False,
        }
        result_file.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    return BridgeInvocation(
        result=result,
        exit_code=completed.returncode,
        result_file=result_file,
        stdout_file=stdout_file,
        stderr_file=stderr_file,
    )


def build_bridge_payload(result: Dict[str, Any]) -> str:
    return RESULT_MARKER + "\n" + json.dumps(result, ensure_ascii=False, indent=2)


def print_turn_header(turn_index: int) -> None:
    print("")
    print(f"========== TURN {turn_index} ==========")


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
        user_message = read_initial_prompt(args)
    except ValueError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2

    instructions = read_instructions(args)

    session_meta = {
        "started_at": datetime.now().isoformat(timespec="seconds"),
        "model": args.model,
        "max_turns": args.max_turns,
        "api_base": args.api_base,
        "reasoning_effort": args.reasoning_effort,
        "session_dir": str(session_dir.resolve()),
        "api_key_source": api_key_source,
    }
    (session_dir / "session_meta.json").write_text(
        json.dumps(session_meta, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (session_dir / "instructions.txt").write_text(instructions + "\n", encoding="utf-8")
    (session_dir / "turn_00_initial_user.txt").write_text(user_message + "\n", encoding="utf-8")

    previous_response_id: Optional[str] = None

    for turn_index in range(1, args.max_turns + 1):
        print_turn_header(turn_index)
        print(f"[INFO] Sending user message to model {args.model}...")

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

        response_file = session_dir / f"turn_{turn_index:02d}_response.json"
        response_file.write_text(json.dumps(response_json, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

        previous_response_id = str(response_json.get("id", "")).strip() or previous_response_id
        assistant_text = extract_assistant_text(response_json)
        if not assistant_text:
            print("[ERROR] Assistant text is empty; see response JSON.", file=sys.stderr)
            return 4

        print("[ASSISTANT]")
        print(assistant_text)

        bridge = run_bridge(
            base_dir=base_dir,
            session_dir=session_dir,
            turn_index=turn_index,
            assistant_text=assistant_text,
            bridge_timeout_s=args.bridge_timeout_s,
        )
        bridge_status = str(bridge.result.get("status", "bridge_error"))

        print("")
        print(f"[BRIDGE] status={bridge_status}, exit_code={bridge.exit_code}")
        print(f"[BRIDGE] result_file={bridge.result_file}")

        if bridge_status == "no_trigger":
            print("")
            print("[DONE] Assistant did not request AUTO_RUN. Loop stopped.")
            print(f"[DONE] Session logs: {session_dir.resolve()}")
            return 0

        user_message = build_bridge_payload(bridge.result)
        next_user_file = session_dir / f"turn_{turn_index:02d}_next_user_payload.txt"
        next_user_file.write_text(user_message + "\n", encoding="utf-8")

        print("[NEXT USER PAYLOAD]")
        print(user_message)

    print("")
    print(f"[DONE] Max turns reached ({args.max_turns}).")
    print(f"[DONE] Session logs: {session_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
