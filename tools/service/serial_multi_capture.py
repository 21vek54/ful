#!/usr/bin/env python3
"""CLI utility for concurrent serial capture from multiple ports.

Simple mode keeps the legacy behavior (keypress stop + optional SEAL EMU AUTO ON).
Scenario mode turns the tool into a bench orchestrator with runtime host commands.
"""

from __future__ import annotations

import argparse
import queue
import re
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Pattern, Set, Tuple

import serial
from serial import SerialException

try:
    import msvcrt
except ImportError:  # pragma: no cover - non-Windows fallback
    msvcrt = None


DEFAULT_BAUDRATE = 115200
DEFAULT_OUTPUT_DIR = Path("logs") / "serial"
DEFAULT_READ_TIMEOUT_S = 0.2
DEFAULT_IDLE_TIMEOUT_S = 30.0
DEFAULT_RESET_PULSE_S = 0.20
DEFAULT_RESET_SETTLE_S = 0.90

ROLE_CONVEYOR = "conveyor"
ROLE_MANIPULATOR = "manipulator"
ROLE_MASTER = "master"

DEFAULT_ROLE_PORTS = {
    ROLE_CONVEYOR: "COM10",
    ROLE_MANIPULATOR: "COM11",
    ROLE_MASTER: "COM12",
}

ROLE_ALIASES = {
    "conv": ROLE_CONVEYOR,
    "conveyor": ROLE_CONVEYOR,
    "manip": ROLE_MANIPULATOR,
    "manipulator": ROLE_MANIPULATOR,
    "master": ROLE_MASTER,
}

CONVEYOR_MOVEMENT_PATTERNS = [
    r"\bP1:",
    r"\bSHIFT:",
    r"\bOTCYCLE:",
    r"\bPOST7\b",
    r"SEAL:\s*cycle complete",
    r"\bSEAL EMU:",
    r"\bC2:",
    r"\bSTEP2\b",
    r"\bP1OT\b",
]

MANIPULATOR_MOVEMENT_PATTERNS = [
    r"\bstep\d+\b",
    r"\bcycle\b",
    r"\bmove\b",
    r"\bmotion\b",
    r"\balarm\b",
    r"\babort\b",
    r"\bfailed\b",
    r"\berror\b",
    r"\bgrip\b",
]

MASTER_MOVEMENT_PATTERNS = [
    r"step3 seen",
    r"step7 reached",
    r"COMMON failed",
    r"abort outcome",
    r"COMMON:\s*(start|pause|abort|pause_|drain|step|cycle)",
    r"refill start requested",
    r"pause requested",
]

STANDARD_SCENARIO_PAUSE_AFTER_S = 60.0
STANDARD_SCENARIO_STOP_AFTER_S = 6 * 60.0
STANDARD_SCENARIO_FAIL_PATTERNS = [
    r"\bCOMMON failed\b",
    r"\bCOMMON:\s*failed\b",
    r"\bmanual_recovery_required\b",
    r"\bmanual recovery required\b",
    r"\babort outcome:\s*failed\b",
    r"\bPOST7:\s*aborted\b",
    r"\bprogram1 aborted\b",
]

STANDARD_READY_MIN_SETTLE_S = 2.5
STANDARD_READY_TIMEOUT_S = 35.0
STANDARD_READY_POLL_S = 0.2
STANDARD_READY_EVIDENCE_MAX_LEN = 140
STANDARD_COM11_PREP_COMMAND = "PREP INIT"
STANDARD_COM11_PREP_TIMEOUT_S = 35.0
STANDARD_COM11_PREP_POLL_S = 0.2
STANDARD_PAUSE_OUTCOME_TIMEOUT_S = 180.0
STANDARD_PAUSE_POST_OUTCOME_TIMEOUT_S = 8.0

PAUSE_OUTCOME_PATTERN = re.compile(
    r"\bpause\s+outcome(?:\s*[:=]\s*|\s+)(?P<outcome>[a-z0-9_/-]+)\b",
    re.IGNORECASE,
)
PAUSE_TERMINAL_STATE_PATTERN = re.compile(
    r"\bpause_state=(?P<state>(?:pause_)?(?:hold|empty|manual_recovery_required))\b",
    re.IGNORECASE,
)

STANDARD_READY_MARKERS_BY_ROLE = {
    ROLE_CONVEYOR: [
        ("i2c_ready", r"\bI2C status:\s*addr\s*12\b"),
    ],
    ROLE_MANIPULATOR: [
        ("i2c_ready", r"\bI2C status ready:\s*addr\s*13\b"),
        ("initial_ready", r"Начальное состояние корректно"),
        ("prep_ready", r"PREP INIT:\s*(ready confirmed|already ready)\b"),
        ("prep_failed", r"PREP INIT:\s*(failed|blocked)\b"),
    ],
    ROLE_MASTER: [
        ("banner", r"===\s*Master controller\s*==="),
        ("common_waiting", r"\bCOMMON:\s*waiting initial batch\b"),
        ("heartbeat_i2c_ok", r"HEARTBEAT:.*i2c_last=read@scan/ok"),
        ("scan_read_12", r"I2C TRACE:\s*start seq=\d+,\s*op=read,\s*id=12,\s*origin=scan"),
    ],
}

RUNTIME_HELP_ITEMS: List[Tuple[str, str]] = [
    ("help | h | ?", "Показать эту справку."),
    ("stop | quit | exit", "Остановить capture и завершить сессию."),
    (
        "reset [all|conveyor|manipulator|master|COMx ...]",
        "Reset выбранных портов (или всех, если all/без аргументов).",
    ),
    ("seal-on", "Отправить на conveyor команду: SEAL EMU AUTO ON."),
    ("plate-pass-on [on_ms] [off_ms]", "Включить INFEED EMU (обычный профиль)."),
    (
        "plate-pass-on realistic [start-on-sensor]",
        "Включить realistic профиль (start-on-sensor - старт с активного датчика).",
    ),
    ("plate-pass-off", "Выключить INFEED EMU."),
    ("plate-pass-status", "Запросить статус INFEED EMU."),
    ("common-start", "Отправить на master команду: COMMON START."),
    ("common-pause", "Отправить на master команду: COMMON PAUSE."),
    ("wait-idle [seconds]", "Подождать тишину по movement/progress событиям (idle)."),
    ("scenario", "Показать список встроенных сценариев."),
    ("scenario prep", "Запустить сценарий prep (reset + staged setup)."),
    (
        "scenario standard",
        "Запустить сценарий standard (readiness + auto-pause/auto-stop).",
    ),
    ("send <role|port> <payload>", "Отправить произвольную команду в роль/порт."),
]


def now_ts() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


def make_session_stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def sanitize_port_for_filename(port: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]", "_", port)


@dataclass
class PortCapture:
    port_name: str
    serial_obj: serial.SerialBase
    file_path: Path
    file_handle: object
    thread: threading.Thread


@dataclass
class StandardScenarioState:
    scenario_label: str
    started_mono: float
    pause_at_mono: Optional[float]
    stop_at_mono: Optional[float]
    fault_detected: bool = False
    fault_ts: str = ""
    fault_event: str = ""
    pause_decision_done: bool = False
    pause_sent: bool = False
    stop_done: bool = False
    pause_outcome_wait_active: bool = False
    pause_outcome_wait_started_mono: Optional[float] = None
    pause_outcome_wait_deadline_mono: Optional[float] = None
    pause_outcome_received: bool = False
    pause_outcome_value: str = ""
    pause_outcome_event: str = ""
    pause_outcome_ts: str = ""
    pause_outcome_post_deadline_mono: Optional[float] = None
    pause_outcome_idle_guard_mono: Optional[float] = None
    pause_outcome_stop_deferred_logged: bool = False


class MultiSerialCapture:
    def __init__(
        self,
        ports: List[str],
        baudrate: int,
        output_dir: Path,
        session_prefix: str,
        read_timeout_s: float,
        duration_s: Optional[float],
        role_ports: Dict[str, str],
        idle_timeout_s: float,
        reset_pulse_s: float,
        reset_settle_s: float,
        auto_stop_on_idle: bool,
        auto_stop_on_idle_after_s: Optional[float],
    ) -> None:
        self.ports = ports
        self.baudrate = baudrate
        self.output_dir = output_dir
        self.session_prefix = session_prefix.strip()
        self.read_timeout_s = read_timeout_s
        self.duration_s = duration_s if duration_s and duration_s > 0 else None

        self.role_ports = role_ports
        self.roles_by_port_upper = self._build_roles_by_port_upper(role_ports)
        self.idle_timeout_s = max(1.0, float(idle_timeout_s))
        self.reset_pulse_s = max(0.05, float(reset_pulse_s))
        self.reset_settle_s = max(0.0, float(reset_settle_s))
        self.auto_stop_on_idle = bool(auto_stop_on_idle)
        self.auto_stop_on_idle_after_s = (
            max(1.0, float(auto_stop_on_idle_after_s))
            if auto_stop_on_idle_after_s is not None
            else self.idle_timeout_s
        )

        self.stop_event = threading.Event()
        self.print_lock = threading.Lock()
        self.captures: Dict[str, PortCapture] = {}
        self.stop_reason: Optional[str] = None

        self.echo_until_mono_by_port: Dict[str, float] = {}
        self.echo_lock = threading.Lock()

        self.session_stamp = make_session_stamp()
        self.session_start_mono: Optional[float] = None

        self.trace_file_path: Optional[Path] = None
        self.trace_file_handle: Optional[object] = None

        self.movement_lock = threading.Lock()
        self.last_movement_mono: Optional[float] = None
        self.last_movement_ts: str = ""
        self.last_movement_event: str = ""
        self.idle_reported_for_event_mono: Optional[float] = None
        self.idle_autostop_reported_for_event_mono: Optional[float] = None

        self.movement_patterns_by_port: Dict[str, List[Pattern[str]]] = self._build_movement_patterns_by_port()
        self.default_movement_patterns = self._compile_patterns(
            CONVEYOR_MOVEMENT_PATTERNS
            + MANIPULATOR_MOVEMENT_PATTERNS
            + MASTER_MOVEMENT_PATTERNS
        )
        self.standard_fail_patterns = self._compile_patterns(STANDARD_SCENARIO_FAIL_PATTERNS)
        self.standard_scenario_lock = threading.Lock()
        self.standard_scenario: Optional[StandardScenarioState] = None

        self.readiness_patterns_by_role = self._build_readiness_patterns_by_role()
        self.readiness_lock = threading.Lock()
        self.readiness_marker_mono: Dict[str, float] = {}
        self.readiness_marker_line: Dict[str, str] = {}

    @staticmethod
    def _compile_patterns(raw_patterns: List[str]) -> List[Pattern[str]]:
        return [re.compile(pattern, re.IGNORECASE) for pattern in raw_patterns]

    def _build_movement_patterns_by_port(self) -> Dict[str, List[Pattern[str]]]:
        mapping: Dict[str, List[Pattern[str]]] = {}
        role_map = {
            ROLE_CONVEYOR: self._compile_patterns(CONVEYOR_MOVEMENT_PATTERNS),
            ROLE_MANIPULATOR: self._compile_patterns(MANIPULATOR_MOVEMENT_PATTERNS),
            ROLE_MASTER: self._compile_patterns(MASTER_MOVEMENT_PATTERNS),
        }
        for role, port_name in self.role_ports.items():
            if not port_name:
                continue
            mapping[port_name.upper()] = role_map.get(role, [])
        return mapping

    @staticmethod
    def _build_roles_by_port_upper(role_ports: Dict[str, str]) -> Dict[str, List[str]]:
        mapping: Dict[str, List[str]] = {}
        for role, port_name in role_ports.items():
            if not port_name:
                continue
            key = port_name.upper()
            mapping.setdefault(key, [])
            if role not in mapping[key]:
                mapping[key].append(role)
        return mapping

    def _build_readiness_patterns_by_role(self) -> Dict[str, List[Tuple[str, Pattern[str]]]]:
        compiled: Dict[str, List[Tuple[str, Pattern[str]]]] = {}
        for role, marker_specs in STANDARD_READY_MARKERS_BY_ROLE.items():
            role_items: List[Tuple[str, Pattern[str]]] = []
            for marker_name, marker_pattern in marker_specs:
                role_items.append((marker_name, re.compile(marker_pattern, re.IGNORECASE)))
            compiled[role] = role_items
        return compiled

    def _console(self, message: str) -> None:
        stamped = f"[{now_ts()}] {message}"
        with self.print_lock:
            print(stamped, flush=True)
            if self.trace_file_handle is not None:
                try:
                    self.trace_file_handle.write(stamped + "\n")
                    self.trace_file_handle.flush()
                except Exception:
                    pass

    def _build_log_path(self, port_name: str) -> Path:
        prefix = f"{self.session_prefix}_" if self.session_prefix else ""
        safe_port = sanitize_port_for_filename(port_name)
        filename = f"{prefix}{self.session_stamp}_{safe_port}.txt"
        return self.output_dir / filename

    def _build_trace_path(self) -> Path:
        prefix = f"{self.session_prefix}_" if self.session_prefix else ""
        filename = f"{prefix}{self.session_stamp}_session_trace.txt"
        return self.output_dir / filename

    def _open_trace_file(self) -> None:
        trace_path = self._build_trace_path()
        try:
            self.trace_file_handle = trace_path.open("w", encoding="utf-8", newline="\n")
            self.trace_file_path = trace_path
        except Exception as exc:
            self.trace_file_handle = None
            self.trace_file_path = None
            self._console(f"[WARN] Не удалось открыть session trace: {exc}")

    def _close_trace_file(self) -> None:
        if self.trace_file_handle is None:
            return
        try:
            self.trace_file_handle.flush()
            self.trace_file_handle.close()
        except Exception:
            pass
        finally:
            self.trace_file_handle = None

    def enable_terminal_echo_for_port(self, port_name: str, duration_s: float) -> None:
        seconds = max(0.1, float(duration_s))
        with self.echo_lock:
            self.echo_until_mono_by_port[port_name] = time.monotonic() + seconds
        self._console(f"[INFO] Ответы {port_name} выводятся в терминал ~{seconds:.1f} c.")

    def _should_echo_to_terminal(self, port_name: str) -> bool:
        with self.echo_lock:
            until_mono = self.echo_until_mono_by_port.get(port_name)
            if until_mono is None:
                return False
            if time.monotonic() > until_mono:
                self.echo_until_mono_by_port.pop(port_name, None)
                return False
            return True

    def _open_port(self, port_name: str) -> Optional[PortCapture]:
        try:
            serial_obj = serial.serial_for_url(
                port_name,
                baudrate=self.baudrate,
                timeout=self.read_timeout_s,
                write_timeout=1.0,
            )
        except Exception as exc:
            self._console(f"[ERROR] Не удалось открыть {port_name}: {exc}")
            return None

        try:
            serial_obj.dtr = True
        except Exception:
            pass

        file_path = self._build_log_path(port_name)
        try:
            file_handle = file_path.open("w", encoding="utf-8", newline="\n")
        except Exception as exc:
            self._console(f"[ERROR] Не удалось открыть файл лога для {port_name}: {exc}")
            try:
                serial_obj.close()
            except Exception:
                pass
            return None

        thread = threading.Thread(
            target=self._reader_loop,
            args=(port_name,),
            name=f"serial_reader_{port_name}",
            daemon=True,
        )

        capture = PortCapture(
            port_name=port_name,
            serial_obj=serial_obj,
            file_path=file_path,
            file_handle=file_handle,
            thread=thread,
        )
        self._console(f"[OPEN] {port_name} @ {self.baudrate} -> {file_path}")
        return capture

    def _record_movement_event(self, event_text: str) -> None:
        with self.movement_lock:
            self.last_movement_mono = time.monotonic()
            self.last_movement_ts = now_ts()
            self.last_movement_event = event_text
            self.idle_reported_for_event_mono = None

    def _line_is_movement(self, port_name: str, line: str) -> bool:
        patterns = self.movement_patterns_by_port.get(port_name.upper(), self.default_movement_patterns)
        for pattern in patterns:
            if pattern.search(line):
                return True
        return False

    def _track_movement_from_line(self, port_name: str, line: str) -> None:
        if not self._line_is_movement(port_name, line):
            return
        self._record_movement_event(f"{port_name}: {line}")

    def _line_is_standard_fail(self, line: str) -> bool:
        for pattern in self.standard_fail_patterns:
            if pattern.search(line):
                return True
        return False

    def _track_standard_scenario_failure(self, port_name: str, line: str) -> None:
        if not self._line_is_standard_fail(line):
            return

        detected = False
        detected_ts = ""
        detected_event = ""
        scenario_label = "standard"
        with self.standard_scenario_lock:
            state = self.standard_scenario
            if state is None:
                return
            if state.fault_detected:
                return
            scenario_label = state.scenario_label
            if state.pause_at_mono is None:
                return
            if time.monotonic() > state.pause_at_mono:
                return
            state.fault_detected = True
            state.fault_ts = now_ts()
            state.fault_event = f"{port_name}: {line}"
            detected = True
            detected_ts = state.fault_ts
            detected_event = state.fault_event

        if detected:
            self._console(
                f"[SCENARIO] {scenario_label}: detected early fault marker before auto-pause deadline, "
                f"auto-pause will be skipped. ts={detected_ts}; event={detected_event}"
            )

    def _start_standard_pause_outcome_wait(self, trigger: str) -> None:
        started = False
        deadline_s = STANDARD_PAUSE_OUTCOME_TIMEOUT_S
        with self.standard_scenario_lock:
            state = self.standard_scenario
            if state is None or state.scenario_label != "standard":
                return
            now_mono = time.monotonic()
            state.pause_outcome_wait_active = True
            state.pause_outcome_wait_started_mono = now_mono
            state.pause_outcome_wait_deadline_mono = now_mono + deadline_s
            state.pause_outcome_received = False
            state.pause_outcome_value = ""
            state.pause_outcome_event = ""
            state.pause_outcome_ts = ""
            state.pause_outcome_post_deadline_mono = None
            state.pause_outcome_idle_guard_mono = None
            state.pause_outcome_stop_deferred_logged = False
            started = True
        if not started:
            return
        self._console(
            "[SCENARIO] standard: pause outcome wait started; "
            f"trigger={trigger}; timeout_s={deadline_s:.0f}; "
            "wait_for=pause outcome=pause_hold|pause_empty|manual_recovery_required"
        )

    @staticmethod
    def _extract_terminal_pause_outcome_from_line(line: str) -> Optional[str]:
        match = PAUSE_OUTCOME_PATTERN.search(line)
        if match:
            value = match.group("outcome").strip().lower()
            if value:
                return value

        match = PAUSE_TERMINAL_STATE_PATTERN.search(line)
        if match:
            state = match.group("state").strip().lower()
            if state == "hold":
                return "pause_hold"
            if state == "empty":
                return "pause_empty"
            if state == "pause_manual_recovery_required":
                return "manual_recovery_required"
            return state

        lowered = line.lower()
        if "manual_recovery_required" in lowered or "manual recovery required" in lowered:
            return "manual_recovery_required"

        return None

    def _track_standard_pause_outcome(self, port_name: str, line: str) -> None:
        master_port = self._role_port(ROLE_MASTER)
        if not master_port or port_name.upper() != master_port.upper():
            return

        outcome_value = self._extract_terminal_pause_outcome_from_line(line)
        if not outcome_value:
            return

        received = False
        event = ""
        received_ts = ""
        post_timeout_s = STANDARD_PAUSE_POST_OUTCOME_TIMEOUT_S
        with self.standard_scenario_lock:
            state = self.standard_scenario
            if state is None or state.scenario_label != "standard":
                return
            if not state.pause_outcome_wait_active or state.pause_outcome_received:
                return

            state.pause_outcome_wait_active = False
            state.pause_outcome_received = True
            state.pause_outcome_value = outcome_value
            state.pause_outcome_ts = now_ts()
            state.pause_outcome_event = f"{port_name}: {line}"
            state.pause_outcome_post_deadline_mono = time.monotonic() + post_timeout_s
            state.pause_outcome_idle_guard_mono = None
            state.pause_outcome_stop_deferred_logged = False
            received = True
            event = state.pause_outcome_event
            received_ts = state.pause_outcome_ts

        if not received:
            return

        self._console(
            "[SCENARIO] standard: pause outcome received: "
            f"{outcome_value}; ts={received_ts}; event={event}; "
            f"post_timeout_s={post_timeout_s:.0f}"
        )

    @staticmethod
    def _readiness_key(role: str, marker_name: str) -> str:
        return f"{role}:{marker_name}"

    @staticmethod
    def _shorten_evidence(line: str) -> str:
        compact = " ".join(line.strip().split())
        if len(compact) <= STANDARD_READY_EVIDENCE_MAX_LEN:
            return compact
        return compact[: STANDARD_READY_EVIDENCE_MAX_LEN - 3] + "..."

    def _track_readiness_from_line(self, port_name: str, line: str) -> None:
        roles = self.roles_by_port_upper.get(port_name.upper(), [])
        if not roles:
            return

        observed_mono = time.monotonic()
        with self.readiness_lock:
            for role in roles:
                patterns = self.readiness_patterns_by_role.get(role, [])
                for marker_name, marker_pattern in patterns:
                    if not marker_pattern.search(line):
                        continue
                    key = self._readiness_key(role, marker_name)
                    self.readiness_marker_mono[key] = observed_mono
                    self.readiness_marker_line[key] = line

    def _get_readiness_marker_since(
        self,
        role: str,
        marker_name: str,
        since_mono: float,
    ) -> Tuple[Optional[float], str]:
        key = self._readiness_key(role, marker_name)
        with self.readiness_lock:
            marker_mono = self.readiness_marker_mono.get(key)
            marker_line = self.readiness_marker_line.get(key, "")
        if marker_mono is None or marker_mono < since_mono:
            return None, ""
        return marker_mono, marker_line

    def _collect_standard_readiness(
        self,
        since_mono: float,
    ) -> Tuple[Dict[str, bool], Dict[str, str]]:
        conv_i2c_mono, conv_i2c_line = self._get_readiness_marker_since(ROLE_CONVEYOR, "i2c_ready", since_mono)
        manip_i2c_mono, manip_i2c_line = self._get_readiness_marker_since(
            ROLE_MANIPULATOR, "i2c_ready", since_mono
        )
        manip_initial_ready_mono, manip_initial_ready_line = self._get_readiness_marker_since(
            ROLE_MANIPULATOR, "initial_ready", since_mono
        )
        master_banner_mono, master_banner_line = self._get_readiness_marker_since(
            ROLE_MASTER, "banner", since_mono
        )
        master_waiting_mono, master_waiting_line = self._get_readiness_marker_since(
            ROLE_MASTER, "common_waiting", since_mono
        )
        master_heartbeat_ok_mono, master_heartbeat_ok_line = self._get_readiness_marker_since(
            ROLE_MASTER, "heartbeat_i2c_ok", since_mono
        )
        master_scan_read_mono, master_scan_read_line = self._get_readiness_marker_since(
            ROLE_MASTER, "scan_read_12", since_mono
        )

        master_console_ready = (master_banner_mono is not None) or (master_waiting_mono is not None)
        master_console_ready_mono: Optional[float] = None
        master_console_ready_line = ""
        if master_banner_mono is not None and (
            master_waiting_mono is None or master_banner_mono <= master_waiting_mono
        ):
            master_console_ready_mono = master_banner_mono
            master_console_ready_line = master_banner_line
        elif master_waiting_mono is not None:
            master_console_ready_mono = master_waiting_mono
            master_console_ready_line = master_waiting_line

        # IMPORTANT: COM12 I2C-ready must be observed only after COM12 console-ready.
        # This avoids accepting stale pre-boot heartbeat/scan lines from a previous runtime.
        master_i2c_ready_mono: Optional[float] = None
        master_i2c_ready_line = ""
        if master_console_ready_mono is not None:
            i2c_candidates: List[Tuple[float, str]] = []
            if master_heartbeat_ok_mono is not None and master_heartbeat_ok_mono >= master_console_ready_mono:
                i2c_candidates.append((master_heartbeat_ok_mono, master_heartbeat_ok_line))
            if master_scan_read_mono is not None and master_scan_read_mono >= master_console_ready_mono:
                i2c_candidates.append((master_scan_read_mono, master_scan_read_line))
            if i2c_candidates:
                i2c_candidates.sort(key=lambda item: item[0])
                master_i2c_ready_mono, master_i2c_ready_line = i2c_candidates[0]

        master_i2c_ready = master_i2c_ready_mono is not None

        evidence: Dict[str, str] = {}
        if conv_i2c_mono is not None:
            evidence["conveyor_ready"] = conv_i2c_line
        if manip_i2c_mono is not None:
            evidence["manipulator_ready"] = manip_i2c_line
        if manip_initial_ready_mono is not None:
            evidence["manipulator_initial_ready"] = manip_initial_ready_line
        if master_console_ready_mono is not None:
            evidence["master_console_ready"] = master_console_ready_line
        if master_i2c_ready_mono is not None:
            evidence["master_i2c_ready"] = master_i2c_ready_line

        status = {
            "conveyor_ready": conv_i2c_mono is not None,
            "manipulator_ready": manip_i2c_mono is not None,
            "manipulator_initial_ready": manip_initial_ready_mono is not None,
            "master_console_ready": master_console_ready,
            "master_i2c_ready": master_i2c_ready,
        }
        status["all_ready"] = (
            status["conveyor_ready"]
            and status["manipulator_ready"]
            and status["master_console_ready"]
            and status["master_i2c_ready"]
        )
        return status, evidence

    def _wait_for_standard_readiness(
        self,
        *,
        reset_started_mono: float,
        settle_s: float = STANDARD_READY_MIN_SETTLE_S,
        timeout_s: float = STANDARD_READY_TIMEOUT_S,
    ) -> bool:
        settle_seconds = max(0.0, float(settle_s))
        timeout_seconds = max(settle_seconds + 1.0, float(timeout_s))
        settle_deadline_mono = reset_started_mono + settle_seconds
        timeout_deadline_mono = reset_started_mono + timeout_seconds

        self._console(
            "[SCENARIO] standard: readiness wait after reset started; "
            f"settle_s={settle_seconds:.1f}; timeout_s={timeout_seconds:.1f}; "
            "need=COM10(i2c_ready)+COM11(i2c_ready)+COM12(console_ready+i2c_ready)"
        )

        while not self.stop_event.is_set() and time.monotonic() < settle_deadline_mono:
            time.sleep(STANDARD_READY_POLL_S)
        if self.stop_event.is_set():
            return False

        self._console("[SCENARIO] standard: readiness settle passed, waiting markers...")
        reported_keys: Set[str] = set()
        marker_labels = [
            ("conveyor_ready", "COM10 ready marker"),
            ("manipulator_ready", "COM11 ready marker"),
            ("master_console_ready", "COM12 console-ready marker"),
            ("master_i2c_ready", "COM12 I2C-ready marker"),
        ]

        while not self.stop_event.is_set() and time.monotonic() < timeout_deadline_mono:
            status, evidence = self._collect_standard_readiness(reset_started_mono)
            for key, label in marker_labels:
                if not status.get(key):
                    continue
                if key in reported_keys:
                    continue
                line = self._shorten_evidence(evidence.get(key, "marker seen"))
                self._console(f"[SCENARIO] standard: readiness observed -> {label}; evidence={line}")
                reported_keys.add(key)

            if status["all_ready"]:
                elapsed_s = time.monotonic() - reset_started_mono
                self._console(
                    "[SCENARIO] standard: readiness confirmed; "
                    f"elapsed_after_reset={elapsed_s:.1f}s. Startup commands will be sent now."
                )
                return True

            time.sleep(STANDARD_READY_POLL_S)

        status, evidence = self._collect_standard_readiness(reset_started_mono)
        self._console(
            "[SCENARIO] standard: readiness wait timeout; proceeding in degraded mode. "
            f"conveyor_ready={status['conveyor_ready']}; "
            f"manipulator_ready={status['manipulator_ready']}; "
            f"master_console_ready={status['master_console_ready']}; "
            f"master_i2c_ready={status['master_i2c_ready']}"
        )
        for key, label in marker_labels:
            line = evidence.get(key)
            if not line:
                continue
            self._console(f"[SCENARIO] standard: readiness evidence {label}: {self._shorten_evidence(line)}")
        return False

    def _wait_for_com11_prep_outcome(
        self,
        *,
        prep_started_mono: float,
        timeout_s: float = STANDARD_COM11_PREP_TIMEOUT_S,
    ) -> bool:
        timeout_seconds = max(1.0, float(timeout_s))
        deadline_mono = prep_started_mono + timeout_seconds
        while not self.stop_event.is_set() and time.monotonic() < deadline_mono:
            ready_mono, ready_line = self._get_readiness_marker_since(
                ROLE_MANIPULATOR,
                "prep_ready",
                prep_started_mono,
            )
            if ready_mono is not None:
                self._console(
                    "[SCENARIO] standard: COM11 prep completed; "
                    f"evidence={self._shorten_evidence(ready_line or 'PREP INIT ready marker')}"
                )
                return True

            failed_mono, failed_line = self._get_readiness_marker_since(
                ROLE_MANIPULATOR,
                "prep_failed",
                prep_started_mono,
            )
            if failed_mono is not None:
                self._console(
                    "[SCENARIO] standard: COM11 prep timeout/failed; "
                    f"evidence={self._shorten_evidence(failed_line or 'PREP INIT failed marker')}"
                )
                return False

            time.sleep(STANDARD_COM11_PREP_POLL_S)

        self._console(
            f"[SCENARIO] standard: COM11 prep timeout/failed; timeout_s={timeout_seconds:.1f}"
        )
        return False

    def _ensure_com11_initial_state_for_standard(
        self,
        *,
        reset_started_mono: float,
    ) -> bool:
        status, evidence = self._collect_standard_readiness(reset_started_mono)
        if status.get("manipulator_initial_ready", False):
            self._console(
                "[SCENARIO] standard: COM11 initial state confirmed before COMMON START; "
                f"evidence={self._shorten_evidence(evidence.get('manipulator_initial_ready', 'marker seen'))}"
            )
            return True

        self._console("[SCENARIO] standard: COM11 not in initial state")
        manip_port = self._role_port(ROLE_MANIPULATOR)
        prep_started_mono = time.monotonic()
        if not self._command_send_with_echo(manip_port, STANDARD_COM11_PREP_COMMAND, echo_s=8.0):
            self._console("[SCENARIO] standard: COM11 prep timeout/failed; command send failed.")
            return False

        self._console("[SCENARIO] standard: COM11 prep started")
        return self._wait_for_com11_prep_outcome(prep_started_mono=prep_started_mono)

    def _tick_standard_scenario(self) -> None:
        pause_action = ""
        pause_fault_ts = ""
        pause_fault_event = ""
        stop_due = False
        stop_deferred_for_pause_wait = False
        pause_wait_timeout_due = False
        pause_wait_elapsed_s = 0.0
        pause_post_timeout_due = False
        scenario_label = "standard"

        with self.standard_scenario_lock:
            state = self.standard_scenario
            if state is None:
                return
            scenario_label = state.scenario_label

            now_mono = time.monotonic()
            if (
                state.pause_at_mono is not None
                and not state.pause_decision_done
                and now_mono >= state.pause_at_mono
            ):
                state.pause_decision_done = True
                if state.fault_detected:
                    pause_action = "skip"
                    pause_fault_ts = state.fault_ts
                    pause_fault_event = state.fault_event
                else:
                    pause_action = "send"

            if (
                state.pause_outcome_wait_active
                and state.pause_outcome_wait_deadline_mono is not None
                and now_mono >= state.pause_outcome_wait_deadline_mono
            ):
                state.pause_outcome_wait_active = False
                pause_wait_timeout_due = True
                if state.pause_outcome_wait_started_mono is not None:
                    pause_wait_elapsed_s = max(0.0, now_mono - state.pause_outcome_wait_started_mono)

            if (
                state.pause_outcome_received
                and state.pause_outcome_post_deadline_mono is not None
                and not state.stop_done
                and now_mono >= state.pause_outcome_post_deadline_mono
            ):
                state.stop_done = True
                pause_post_timeout_due = True

            if (
                state.stop_at_mono is not None
                and not state.stop_done
                and now_mono >= state.stop_at_mono
            ):
                if state.pause_outcome_wait_active:
                    if not state.pause_outcome_stop_deferred_logged:
                        state.pause_outcome_stop_deferred_logged = True
                        stop_deferred_for_pause_wait = True
                else:
                    state.stop_done = True
                    stop_due = True

        if pause_action == "send":
            master_port = self._role_port(ROLE_MASTER)
            sent = self._command_send_with_echo(master_port, "COMMON PAUSE", echo_s=8.0)
            with self.standard_scenario_lock:
                state = self.standard_scenario
                if state is not None:
                    state.pause_sent = sent
            if sent:
                self._console(
                    f"[SCENARIO] {scenario_label}: healthy runtime window reached, COMMON PAUSE sent."
                )
                if scenario_label == "standard":
                    self._start_standard_pause_outcome_wait(trigger="scenario-auto-common-pause")
            else:
                self._console(
                    f"[SCENARIO] {scenario_label}: reached auto-pause deadline, but COMMON PAUSE send FAILED."
                )
        elif pause_action == "skip":
            self._console(
                f"[SCENARIO] {scenario_label}: COMMON PAUSE skipped due to early fault marker; "
                f"fault_ts={pause_fault_ts}; fault_event={pause_fault_event}"
            )

        if stop_deferred_for_pause_wait:
            self._console(
                "[SCENARIO] standard: auto-stop deadline reached, "
                "but deferred while waiting terminal pause outcome from COM12."
            )

        if pause_wait_timeout_due:
            self.stop_reason = "standard-pause-outcome-timeout"
            self._console(
                "[SCENARIO] standard: pause outcome wait timeout; "
                f"waited_s={pause_wait_elapsed_s:.1f}; action=stop"
            )
            self.stop_event.set()
            return

        if pause_post_timeout_due:
            self.stop_reason = "standard-pause-outcome-post-timeout"
            self._console(
                "[SCENARIO] standard: post-outcome timeout reached, stopping capture."
            )
            self.stop_event.set()
            return

        if stop_due:
            self.stop_reason = f"scenario-{scenario_label}-autostop"
            self._console(f"[SCENARIO] {scenario_label}: auto-stop deadline reached, stopping capture.")
            self.stop_event.set()

    def _tick_idle_autostop(self) -> None:
        if self.stop_event.is_set():
            return
        if not self.auto_stop_on_idle:
            return
        threshold_s = self.auto_stop_on_idle_after_s
        if threshold_s <= 0:
            return
        if not self._check_idle_transition(threshold_s):
            return

        pause_wait_active = False
        pause_wait_log = False
        pause_wait_remaining_s: Optional[float] = None
        with self.movement_lock:
            last_movement_mono = self.last_movement_mono
            last_ts = self.last_movement_ts
            last_event = self.last_movement_event

        with self.standard_scenario_lock:
            state = self.standard_scenario
            if (
                state is not None
                and state.scenario_label == "standard"
                and state.pause_outcome_wait_active
            ):
                pause_wait_active = True
                if state.pause_outcome_wait_deadline_mono is not None:
                    pause_wait_remaining_s = max(0.0, state.pause_outcome_wait_deadline_mono - time.monotonic())
                if state.pause_outcome_idle_guard_mono != last_movement_mono:
                    state.pause_outcome_idle_guard_mono = last_movement_mono
                    pause_wait_log = True

        if pause_wait_active:
            if pause_wait_log:
                remaining_text = (
                    f"{pause_wait_remaining_s:.1f}s"
                    if pause_wait_remaining_s is not None
                    else "unknown"
                )
                self._console(
                    "[AUTO-STOP] idle auto-stop deferred: "
                    "waiting terminal pause outcome from COM12; "
                    f"pause_timeout_in={remaining_text}; "
                    f"last_movement_ts={last_ts}; last_event={last_event}"
                )
            return

        should_stop = False
        with self.movement_lock:
            if self.idle_autostop_reported_for_event_mono != self.last_movement_mono:
                self.idle_autostop_reported_for_event_mono = self.last_movement_mono
                should_stop = True
                last_ts = self.last_movement_ts
                last_event = self.last_movement_event

        if not should_stop:
            return

        stop_ts = now_ts()
        self.stop_reason = "auto-stop-on-idle"
        self._console(
            "[AUTO-STOP] idle auto-stop triggered: "
            f"no movement/progress for {threshold_s:.0f}s; stop_ts={stop_ts}; "
            f"last_movement_ts={last_ts}; last_event={last_event}"
        )
        self.stop_event.set()

    def _tick_runtime_automation(self, allow_idle_autostop: bool = True) -> None:
        self._tick_standard_scenario()
        if allow_idle_autostop and not self.stop_event.is_set():
            self._tick_idle_autostop()

    def _check_idle_transition(self, idle_for_s: Optional[float] = None) -> bool:
        threshold_s = self.idle_timeout_s if idle_for_s is None else max(1.0, float(idle_for_s))
        with self.movement_lock:
            if self.last_movement_mono is None:
                return False
            now_mono = time.monotonic()
            idle_elapsed_s = now_mono - self.last_movement_mono
            if idle_elapsed_s < threshold_s:
                return False
            if self.idle_reported_for_event_mono == self.last_movement_mono:
                return True
            self.idle_reported_for_event_mono = self.last_movement_mono
            last_ts = self.last_movement_ts
            last_event = self.last_movement_event

        self._console(
            "[IDLE] Система без movement/progress уже "
            f"{threshold_s:.0f} c; last_movement_ts={last_ts}; last_event={last_event}"
        )
        return True

    def wait_for_idle(self, idle_for_s: Optional[float] = None) -> bool:
        threshold_s = self.idle_timeout_s if idle_for_s is None else max(1.0, float(idle_for_s))
        self._console(f"[WAIT] Ожидаю состояние idle >= {threshold_s:.0f} c...")
        while not self.stop_event.is_set():
            self._tick_runtime_automation(allow_idle_autostop=False)
            if self._check_idle_transition(threshold_s):
                return True

            if self.duration_s is not None and self.session_start_mono is not None:
                if (time.monotonic() - self.session_start_mono) >= self.duration_s:
                    self._console("[WAIT] Прервано: достигнут лимит времени сессии.")
                    return False

            time.sleep(0.2)

        self._console("[WAIT] Прервано: сессия остановлена.")
        return False

    def start(self) -> int:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._open_trace_file()
        self._console(f"[INFO] Папка логов: {self.output_dir.resolve()}")
        if self.trace_file_path is not None:
            self._console(f"[INFO] Session trace: {self.trace_file_path}")

        for port_name in self.ports:
            capture = self._open_port(port_name)
            if capture is None:
                continue
            self.captures[port_name] = capture

        if not self.captures:
            self._console("[ERROR] Ни один порт не открылся. Сессия завершена.")
            self._close_trace_file()
            return 1

        for capture in self.captures.values():
            capture.thread.start()

        self.session_start_mono = time.monotonic()
        self._record_movement_event("HOST: capture-start")

        self._console(f"[INFO] Логирование началось: {now_ts()}")
        if self.duration_s is not None:
            self._console(f"[INFO] Автостоп через {self.duration_s:.1f} c.")
        return 0

    def _poll_stop_keypress(self) -> None:
        if msvcrt is None:
            return
        if not sys.stdin.isatty():
            return
        if not msvcrt.kbhit():
            return

        pressed = msvcrt.getwch()
        if pressed in ("\x00", "\xe0") and msvcrt.kbhit():
            msvcrt.getwch()

        self.stop_reason = "keypress"
        self._console("[INFO] Получен символ в терминале, останавливаю сессию...")
        self.stop_event.set()

    def wait(self, allow_keypress_stop: bool = True) -> None:
        if allow_keypress_stop:
            self._console("[INFO] Нажмите любую клавишу в терминале или Ctrl+C для остановки.")
        else:
            self._console("[INFO] Ожидание без keypress-stop (только Ctrl+C/host-команды/таймаут).")

        while not self.stop_event.is_set():
            if allow_keypress_stop:
                self._poll_stop_keypress()
                if self.stop_event.is_set():
                    break

            self._tick_runtime_automation()
            self._check_idle_transition()

            if self.duration_s is not None and self.session_start_mono is not None:
                if (time.monotonic() - self.session_start_mono) >= self.duration_s:
                    self._console("[INFO] Достигнут лимит времени сессии.")
                    break

            alive = any(capture.thread.is_alive() for capture in self.captures.values())
            if not alive:
                self._console("[WARN] Все потоки чтения остановлены. Сессия завершится.")
                break

            time.sleep(0.2)

    def _reader_loop(self, port_name: str) -> None:
        capture = self.captures[port_name]
        serial_obj = capture.serial_obj
        file_handle = capture.file_handle

        try:
            while not self.stop_event.is_set():
                try:
                    raw = serial_obj.readline()
                except SerialException as exc:
                    self._console(f"[ERROR] Порт {port_name} отвалился: {exc}")
                    break
                except Exception as exc:
                    self._console(f"[ERROR] Ошибка чтения {port_name}: {exc}")
                    break

                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                stamped = f"[{now_ts()}] {line}\n"
                try:
                    file_handle.write(stamped)
                    file_handle.flush()
                except Exception as exc:
                    self._console(f"[ERROR] Ошибка записи файла для {port_name}: {exc}")
                    break

                self._track_readiness_from_line(port_name, line)
                self._track_movement_from_line(port_name, line)
                self._track_standard_scenario_failure(port_name, line)
                self._track_standard_pause_outcome(port_name, line)

                if self._should_echo_to_terminal(port_name):
                    self._console(f"[{port_name}] {line}")
        finally:
            self._console(f"[CLOSE] Поток {port_name} остановлен.")

    def stop(self) -> None:
        self.stop_event.set()

        for capture in self.captures.values():
            capture.thread.join(timeout=self.read_timeout_s + 1.0)

        for capture in self.captures.values():
            try:
                capture.serial_obj.close()
            except Exception:
                pass
            try:
                capture.file_handle.close()
            except Exception:
                pass

        self._console("[INFO] Все порты и файлы закрыты.")
        self._close_trace_file()

    def send_command(self, port_name: str, command: str) -> bool:
        capture = self.captures.get(port_name)
        if capture is None:
            self._console(f"[WARN] Порт {port_name} не открыт, команда не отправлена: {command}")
            return False

        payload = (command.strip() + "\n").encode("utf-8")
        try:
            capture.serial_obj.write(payload)
            capture.serial_obj.flush()
        except Exception as exc:
            self._console(f"[ERROR] Не удалось отправить команду в {port_name}: {exc}")
            return False

        self._console(f"[CMD] {port_name} <= {command.strip()}")
        return True

    def _resolve_port_from_token(self, token: str) -> Optional[str]:
        normalized = token.strip().lower()
        if not normalized:
            return None

        role = ROLE_ALIASES.get(normalized)
        if role is not None:
            return self.role_ports.get(role)

        expanded = expand_port_name(token)
        if expanded in self.captures:
            return expanded

        if expanded.upper() in self.captures:
            return expanded.upper()

        return None

    def _ordered_open_ports(self) -> List[str]:
        ordered: List[str] = []
        for role in (ROLE_CONVEYOR, ROLE_MANIPULATOR, ROLE_MASTER):
            port_name = self.role_ports.get(role)
            if port_name and port_name in self.captures and port_name not in ordered:
                ordered.append(port_name)

        for port_name in self.captures.keys():
            if port_name not in ordered:
                ordered.append(port_name)

        return ordered

    def reset_port(self, port_name: str) -> bool:
        capture = self.captures.get(port_name)
        if capture is None:
            self._console(f"[WARN] Порт {port_name} не открыт, reset пропущен.")
            return False

        serial_obj = capture.serial_obj
        try:
            old_dtr = bool(serial_obj.dtr)
        except Exception:
            old_dtr = True

        try:
            old_rts = bool(serial_obj.rts)
        except Exception:
            old_rts = True

        self._console(f"[RESET] {port_name}: DTR/RTS pulse started.")
        try:
            try:
                serial_obj.dtr = (not old_dtr)
            except Exception:
                pass
            try:
                serial_obj.rts = (not old_rts)
            except Exception:
                pass

            time.sleep(self.reset_pulse_s)

            try:
                serial_obj.dtr = old_dtr
            except Exception:
                pass
            try:
                serial_obj.rts = old_rts
            except Exception:
                pass
        except Exception as exc:
            self._console(f"[RESET] {port_name}: FAILED ({exc})")
            return False

        self.enable_terminal_echo_for_port(port_name, duration_s=max(4.0, self.reset_settle_s + 3.0))
        self._console(f"[RESET] {port_name}: OK")
        if self.reset_settle_s > 0:
            time.sleep(self.reset_settle_s)
        return True

    def reset_ports(self, port_names: List[str]) -> bool:
        if not port_names:
            self._console("[WARN] Нет портов для reset.")
            return False

        ok = True
        for port_name in port_names:
            if not self.reset_port(port_name):
                ok = False

        if ok:
            self._console("[RESET] Групповой reset завершен успешно.")
        else:
            self._console("[RESET] Групповой reset завершен с ошибками.")
        return ok

    def _role_port(self, role: str) -> Optional[str]:
        port_name = self.role_ports.get(role)
        if port_name in self.captures:
            return port_name
        return port_name

    def _command_send_with_echo(self, port_name: Optional[str], payload: str, echo_s: float = 6.0) -> bool:
        if not port_name:
            self._console(f"[WARN] Не задан порт для команды: {payload}")
            return False
        self.enable_terminal_echo_for_port(port_name, duration_s=echo_s)
        return self.send_command(port_name, payload)

    def run_scenario_prep(
        self,
        *,
        seal_emu_auto_on: bool = True,
        plate_profile: str = "generic",
        start_on_sensor: bool = False,
        auto_common_start: bool = False,
        auto_common_pause_after_s: Optional[float] = None,
        auto_stop_after_s: Optional[float] = None,
    ) -> None:
        self._clear_standard_scenario("scenario prep started")
        self._console("[SCENARIO] prep: reset + staged setup.")
        self.reset_ports(self._ordered_open_ports())
        conveyor_port = self._role_port(ROLE_CONVEYOR)
        if seal_emu_auto_on:
            self._command_send_with_echo(conveyor_port, "SEAL EMU AUTO ON", echo_s=8.0)
        self._apply_plate_profile(
            plate_profile=plate_profile,
            start_on_sensor=start_on_sensor,
            echo_s=8.0,
        )
        if auto_common_start:
            master_port = self._role_port(ROLE_MASTER)
            self._command_send_with_echo(master_port, "COMMON START", echo_s=8.0)
        self._arm_scenario_scheduler(
            label="prep",
            auto_common_pause_after_s=auto_common_pause_after_s,
            auto_stop_after_s=auto_stop_after_s,
        )
        self._record_movement_event("HOST: scenario-prep-start")

    @staticmethod
    def _build_realistic_plate_payload(start_on_sensor: bool = False) -> str:
        if start_on_sensor:
            return "INFEED EMU REALISTIC ON START_ON_SENSOR"
        return "INFEED EMU REALISTIC ON"

    def _apply_plate_profile(self, plate_profile: str, start_on_sensor: bool, echo_s: float) -> bool:
        conveyor_port = self._role_port(ROLE_CONVEYOR)
        profile = plate_profile.strip().lower()
        if profile in ("", "none", "off"):
            self._console("[SCENARIO] plate profile skipped.")
            return True
        if profile in ("realistic", "real"):
            payload = self._build_realistic_plate_payload(start_on_sensor=start_on_sensor)
            return self._command_send_with_echo(conveyor_port, payload, echo_s=echo_s)
        if profile in ("generic", "on"):
            return self._command_send_with_echo(conveyor_port, "INFEED EMU ON", echo_s=echo_s)
        self._console(f"[WARN] Unknown plate profile: {plate_profile}")
        return False

    def _arm_scenario_scheduler(
        self,
        *,
        label: str,
        auto_common_pause_after_s: Optional[float],
        auto_stop_after_s: Optional[float],
    ) -> None:
        pause_after_s = (
            float(auto_common_pause_after_s)
            if auto_common_pause_after_s is not None and auto_common_pause_after_s > 0
            else None
        )
        stop_after_s = (
            float(auto_stop_after_s)
            if auto_stop_after_s is not None and auto_stop_after_s > 0
            else None
        )
        if pause_after_s is None and stop_after_s is None:
            return

        started_mono = time.monotonic()
        state = StandardScenarioState(
            scenario_label=label,
            started_mono=started_mono,
            pause_at_mono=(started_mono + pause_after_s) if pause_after_s is not None else None,
            stop_at_mono=(started_mono + stop_after_s) if stop_after_s is not None else None,
        )
        with self.standard_scenario_lock:
            self.standard_scenario = state
        self._console(
            f"[SCENARIO] {label}: scheduler armed; "
            f"auto_pause_after_s={pause_after_s if pause_after_s is not None else 'off'}; "
            f"auto_stop_after_s={stop_after_s if stop_after_s is not None else 'off'}"
        )

    def _clear_standard_scenario(self, reason: str) -> None:
        had_state = False
        label = "standard"
        with self.standard_scenario_lock:
            if self.standard_scenario is not None:
                label = self.standard_scenario.scenario_label
                had_state = True
            self.standard_scenario = None
        if had_state:
            self._console(f"[SCENARIO] {label}: canceled ({reason}).")

    def run_scenario_standard(
        self,
        *,
        seal_emu_auto_on: bool = True,
        plate_profile: str = "realistic",
        start_on_sensor: bool = True,
        auto_common_start: bool = True,
        auto_common_pause_after_s: Optional[float] = STANDARD_SCENARIO_PAUSE_AFTER_S,
        auto_stop_after_s: Optional[float] = STANDARD_SCENARIO_STOP_AFTER_S,
    ) -> None:
        self._clear_standard_scenario("scenario standard restarted")
        started_ts = now_ts()
        self._console(
            "[SCENARIO] standard: start requested "
            "(reset all -> optional setup -> optional COMMON START -> scheduler)."
        )

        if (
            auto_stop_after_s is not None
            and auto_stop_after_s > 0
            and self.duration_s is not None
            and self.duration_s < auto_stop_after_s
        ):
            self._console(
                "[SCENARIO] standard: warning, --duration-s is shorter than scenario auto-stop; "
                "external timeout may stop the session earlier."
            )

        reset_started_mono = time.monotonic()
        self.reset_ports(self._ordered_open_ports())
        readiness_ok = self._wait_for_standard_readiness(reset_started_mono=reset_started_mono)
        if self.stop_event.is_set():
            self._console("[SCENARIO] standard: interrupted during readiness wait.")
            return
        if not readiness_ok:
            self._console(
                "[SCENARIO] standard: readiness was not fully confirmed before timeout; "
                "continuing startup commands."
            )

        com11_prep_ok = self._ensure_com11_initial_state_for_standard(
            reset_started_mono=reset_started_mono
        )
        if self.stop_event.is_set():
            self._console("[SCENARIO] standard: interrupted during COM11 prep wait.")
            return
        if not com11_prep_ok:
            self._console(
                "[SCENARIO] standard: startup canceled; COM11 prep did not finish successfully."
            )
            return

        conveyor_port = self._role_port(ROLE_CONVEYOR)
        master_port = self._role_port(ROLE_MASTER)

        if seal_emu_auto_on:
            seal_sent = self._command_send_with_echo(conveyor_port, "SEAL EMU AUTO ON", echo_s=8.0)
            if seal_sent:
                self._console("[SCENARIO] standard: SEAL EMU AUTO ON enabled on conveyor.")
            else:
                self._console("[SCENARIO] standard: failed to enable SEAL EMU AUTO ON on conveyor.")

        plate_sent = self._apply_plate_profile(
            plate_profile=plate_profile,
            start_on_sensor=start_on_sensor,
            echo_s=10.0,
        )
        if plate_profile.strip().lower() in ("realistic", "real"):
            if plate_sent:
                self._console(
                    "[SCENARIO] standard: realistic plate profile enabled "
                    f"(start_on_sensor={'yes' if start_on_sensor else 'no'})."
                )
            else:
                self._console("[SCENARIO] standard: failed to enable realistic plate profile.")

        if auto_common_start:
            common_start_sent = self._command_send_with_echo(master_port, "COMMON START", echo_s=8.0)
            if common_start_sent:
                self._console("[SCENARIO] standard: COMMON START sent.")
            else:
                self._console("[SCENARIO] standard: COMMON START send FAILED.")

        self._arm_scenario_scheduler(
            label="standard",
            auto_common_pause_after_s=auto_common_pause_after_s,
            auto_stop_after_s=auto_stop_after_s,
        )

        self._record_movement_event("HOST: scenario-standard-start")
        self._console(
            "[SCENARIO] standard: active. "
            f"started_ts={started_ts}; "
            f"auto_pause_after_s={auto_common_pause_after_s if auto_common_pause_after_s is not None else 'off'}; "
            f"auto_stop_after_s={auto_stop_after_s if auto_stop_after_s is not None else 'off'}"
        )

    def _print_runtime_help(self) -> None:
        self._console("[HELP] Доступные host-команды (вводите после prompt host>):")
        for cmd, description in RUNTIME_HELP_ITEMS:
            self._console(f"[HELP]   {cmd}")
            self._console(f"[HELP]      {description}")
        self._console(
            "[HELP]      scenario standard defaults: "
            f"auto_pause_after_s={int(STANDARD_SCENARIO_PAUSE_AFTER_S)}; "
            f"auto_stop_after_s={int(STANDARD_SCENARIO_STOP_AFTER_S)}"
        )
        self._console("[HELP]      role aliases: conv|conveyor, manip|manipulator, master, COMx.")

    def _handle_host_command(self, raw_line: str) -> None:
        line = raw_line.strip()
        if not line:
            return

        parts = line.split()
        command = parts[0].lower()
        args = parts[1:]

        if command in ("help", "h", "?"):
            self._print_runtime_help()
            return

        if command in ("stop", "quit", "exit"):
            self.stop_reason = "host-stop"
            self._console("[HOST] stop requested.")
            self.stop_event.set()
            return

        if command == "reset":
            if not args or args[0].lower() == "all":
                self.reset_ports(self._ordered_open_ports())
                return

            targets: List[str] = []
            for token in args:
                port_name = self._resolve_port_from_token(token)
                if not port_name:
                    self._console(f"[WARN] Неизвестный target для reset: {token}")
                    continue
                if port_name not in targets:
                    targets.append(port_name)

            self.reset_ports(targets)
            return

        if command == "seal-on":
            conveyor_port = self._role_port(ROLE_CONVEYOR)
            self._command_send_with_echo(conveyor_port, "SEAL EMU AUTO ON", echo_s=8.0)
            return

        if command == "plate-pass-on":
            conveyor_port = self._role_port(ROLE_CONVEYOR)
            payload = "INFEED EMU ON"
            if args and args[0].lower() in ("realistic", "real"):
                start_on_sensor = False
                if len(args) >= 2:
                    option = args[1].lower()
                    if option in ("start-on-sensor", "with-plate", "sensor-active", "hot-start"):
                        start_on_sensor = True
                    elif option in ("normal", "no-plate", "clear-start"):
                        start_on_sensor = False
                    else:
                        self._console(f"[WARN] Unknown realistic mode: {args[1]}")
                        return
                payload = self._build_realistic_plate_payload(start_on_sensor=start_on_sensor)
            elif len(args) == 1:
                payload = f"INFEED EMU ON {args[0]}"
            elif len(args) >= 2:
                payload = f"INFEED EMU ON {args[0]} {args[1]}"
            self._command_send_with_echo(conveyor_port, payload, echo_s=8.0)
            return

        if command == "plate-pass-off":
            conveyor_port = self._role_port(ROLE_CONVEYOR)
            self._command_send_with_echo(conveyor_port, "INFEED EMU OFF", echo_s=4.0)
            return

        if command == "plate-pass-status":
            conveyor_port = self._role_port(ROLE_CONVEYOR)
            self._command_send_with_echo(conveyor_port, "INFEED EMU STATUS", echo_s=5.0)
            return

        if command == "common-start":
            master_port = self._role_port(ROLE_MASTER)
            self._command_send_with_echo(master_port, "COMMON START", echo_s=6.0)
            return

        if command == "common-pause":
            master_port = self._role_port(ROLE_MASTER)
            sent = self._command_send_with_echo(master_port, "COMMON PAUSE", echo_s=6.0)
            if sent:
                self._start_standard_pause_outcome_wait(trigger="host-common-pause")
            return

        if command == "wait-idle":
            wait_seconds = self.idle_timeout_s
            if args:
                try:
                    wait_seconds = float(args[0])
                except ValueError:
                    self._console(f"[WARN] Некорректное значение seconds: {args[0]}")
                    return
            self.wait_for_idle(wait_seconds)
            return

        if command == "scenario":
            if not args:
                self._console("[HOST] Available scenarios: prep, standard")
                return
            scenario_name = args[0].lower()
            if scenario_name == "prep":
                self.run_scenario_prep()
                return
            if scenario_name == "standard":
                self.run_scenario_standard()
                return
            self._console(f"[WARN] Неизвестный сценарий: {scenario_name}")
            return

        if command == "send":
            if len(args) < 2:
                self._console("[WARN] Usage: send <role|port> <payload>")
                return
            port_name = self._resolve_port_from_token(args[0])
            if not port_name:
                self._console(f"[WARN] Неизвестный target: {args[0]}")
                return
            payload = " ".join(args[1:])
            self._command_send_with_echo(port_name, payload, echo_s=5.0)
            return

        self._console(f"[WARN] Неизвестная host-команда: {line}")

    def run_command_script(self, script_path: Path) -> bool:
        try:
            raw_lines = script_path.read_text(encoding="utf-8").splitlines()
        except Exception as exc:
            self._console(f"[ERROR] Не удалось прочитать command script {script_path}: {exc}")
            return False

        self._console(f"[SCRIPT] Выполняю command script: {script_path}")
        for idx, raw_line in enumerate(raw_lines, start=1):
            if self.stop_event.is_set():
                self._console("[SCRIPT] Остановлено: stop_event уже выставлен.")
                return False
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            self._console(f"[SCRIPT] line {idx}: {line}")
            self._handle_host_command(line)
        self._console("[SCRIPT] Command script завершен.")
        return True

    def _stdin_reader_loop(self, command_queue: "queue.Queue[str]") -> None:
        while not self.stop_event.is_set():
            try:
                line = input("host> ")
            except EOFError:
                command_queue.put("stop")
                return
            except KeyboardInterrupt:
                command_queue.put("stop")
                return
            command_queue.put(line)

    def run_command_console(self) -> None:
        if not sys.stdin.isatty():
            self._console("[WARN] stdin не интерактивный, runtime-консоль недоступна.")
            self.wait(allow_keypress_stop=False)
            return

        command_queue: "queue.Queue[str]" = queue.Queue()
        input_thread = threading.Thread(
            target=self._stdin_reader_loop,
            args=(command_queue,),
            name="host_command_reader",
            daemon=True,
        )
        input_thread.start()

        self._console("[INFO] Runtime-консоль активна. Для остановки: stop")

        while not self.stop_event.is_set():
            self._tick_runtime_automation()
            self._check_idle_transition()

            if self.duration_s is not None and self.session_start_mono is not None:
                if (time.monotonic() - self.session_start_mono) >= self.duration_s:
                    self._console("[INFO] Достигнут лимит времени сессии.")
                    break

            alive = any(capture.thread.is_alive() for capture in self.captures.values())
            if not alive:
                self._console("[WARN] Все потоки чтения остановлены. Сессия завершится.")
                break

            try:
                raw_line = command_queue.get(timeout=0.2)
            except queue.Empty:
                continue

            self._handle_host_command(raw_line)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Снятие логов с нескольких COM-портов в отдельные файлы.",
    )
    parser.epilog = (
        "Scenario standard defaults: "
        f"auto_common_pause_after_s={int(STANDARD_SCENARIO_PAUSE_AFTER_S)}; "
        f"auto_stop_after_s={int(STANDARD_SCENARIO_STOP_AFTER_S)}"
    )
    parser.add_argument(
        "ports",
        nargs="*",
        help="Список портов/URL pyserial. Если не указано, скрипт спросит их при старте.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f"Скорость порта (по умолчанию: {DEFAULT_BAUDRATE})",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help=f"Папка для логов (по умолчанию: {DEFAULT_OUTPUT_DIR})",
    )
    parser.add_argument(
        "--session-prefix",
        default="",
        help="Опциональный префикс имени сессии в файлах.",
    )
    parser.add_argument(
        "--read-timeout-s",
        type=float,
        default=DEFAULT_READ_TIMEOUT_S,
        help=f"Таймаут чтения serial (по умолчанию: {DEFAULT_READ_TIMEOUT_S})",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=0.0,
        help="Опциональный автостоп в секундах (0 = только Ctrl+C).",
    )
    parser.add_argument(
        "--scenario-mode",
        action="store_true",
        help="Включить runtime host-консоль (orchestrator mode).",
    )
    parser.add_argument(
        "--scenario",
        choices=("prep", "standard"),
        default=None,
        help="Запустить сценарий сразу после старта capture (без ручного host> ввода).",
    )
    parser.add_argument(
        "--seal-emu-auto-on",
        action="store_true",
        help="В сценарии включить SEAL EMU AUTO ON на conveyor.",
    )
    parser.add_argument(
        "--plate-profile",
        choices=("none", "generic", "realistic"),
        default=None,
        help="Профиль plate emulation для сценария.",
    )
    parser.add_argument(
        "--start-on-sensor",
        action="store_true",
        help="Для realistic plate profile стартовать как будто тарелка уже на датчике.",
    )
    parser.add_argument(
        "--auto-common-start",
        action="store_true",
        help="В сценарии автоматически отправить COMMON START на master.",
    )
    parser.add_argument(
        "--auto-common-pause-after-s",
        type=float,
        default=None,
        help="Авто-отправка COMMON PAUSE через N секунд от старта сценария.",
    )
    parser.add_argument(
        "--auto-stop-after-s",
        type=float,
        default=None,
        help="Авто-остановка capture через N секунд от старта сценария.",
    )
    parser.add_argument(
        "--auto-stop-on-idle",
        action="store_true",
        help="Остановить capture, если нет movement/progress событий дольше idle порога.",
    )
    parser.add_argument(
        "--auto-stop-on-idle-after-s",
        type=float,
        default=None,
        help="Порог idle для авто-остановки в секундах (если не задан, используется --idle-timeout-s).",
    )
    parser.add_argument(
        "--command-script",
        type=Path,
        default=None,
        help="Файл host-команд (по одной на строку), выполняется без интерактивной консоли.",
    )
    parser.add_argument(
        "--conveyor-port",
        default=DEFAULT_ROLE_PORTS[ROLE_CONVEYOR],
        help=f"Порт роли conveyor (по умолчанию: {DEFAULT_ROLE_PORTS[ROLE_CONVEYOR]}).",
    )
    parser.add_argument(
        "--manipulator-port",
        default=DEFAULT_ROLE_PORTS[ROLE_MANIPULATOR],
        help=f"Порт роли manipulator (по умолчанию: {DEFAULT_ROLE_PORTS[ROLE_MANIPULATOR]}).",
    )
    parser.add_argument(
        "--master-port",
        default=DEFAULT_ROLE_PORTS[ROLE_MASTER],
        help=f"Порт роли master (по умолчанию: {DEFAULT_ROLE_PORTS[ROLE_MASTER]}).",
    )
    parser.add_argument(
        "--idle-timeout-s",
        type=float,
        default=DEFAULT_IDLE_TIMEOUT_S,
        help=f"Порог idle детектора по movement/progress (по умолчанию: {DEFAULT_IDLE_TIMEOUT_S}).",
    )
    parser.add_argument(
        "--reset-pulse-s",
        type=float,
        default=DEFAULT_RESET_PULSE_S,
        help=f"Длительность DTR/RTS pulse для reset (по умолчанию: {DEFAULT_RESET_PULSE_S}).",
    )
    parser.add_argument(
        "--reset-settle-s",
        type=float,
        default=DEFAULT_RESET_SETTLE_S,
        help=f"Пауза после reset порта (по умолчанию: {DEFAULT_RESET_SETTLE_S}).",
    )
    for action in parser._actions:
        if "--auto-common-pause-after-s" in action.option_strings:
            action.help = (
                "Auto-send COMMON PAUSE after N seconds from scenario start. "
                f"For scenario=standard default is {int(STANDARD_SCENARIO_PAUSE_AFTER_S)}."
            )
            break
    return parser.parse_args()


def parse_ports_text(raw_text: str) -> List[str]:
    items = re.split(r"[\s,;]+", raw_text.strip())
    return [item for item in items if item]


def expand_port_name(raw_name: str) -> str:
    token = raw_name.strip().lstrip("\ufeff")
    if not token:
        return token

    if "://" in token:
        return token

    if re.fullmatch(r"\d+", token):
        return f"COM{token}"

    match = re.fullmatch(r"(?i)com(\d+)", token)
    if match:
        return f"COM{match.group(1)}"

    return token


def normalize_ports(raw_ports: List[str]) -> List[str]:
    normalized: List[str] = []
    seen = set()
    for raw_name in raw_ports:
        port_name = expand_port_name(raw_name)
        if not port_name:
            continue
        if port_name in seen:
            print(f"[WARN] Дубликат порта пропущен: {port_name}", flush=True)
            continue
        seen.add(port_name)
        normalized.append(port_name)
    return normalized


def normalize_role_ports(args: argparse.Namespace) -> Dict[str, str]:
    return {
        ROLE_CONVEYOR: expand_port_name(args.conveyor_port),
        ROLE_MANIPULATOR: expand_port_name(args.manipulator_port),
        ROLE_MASTER: expand_port_name(args.master_port),
    }


def prompt_ports_if_needed(raw_ports: List[str]) -> List[str]:
    if raw_ports:
        return raw_ports

    while True:
        text = input("Какие порты логировать? Пример: 10,12 или COM10 COM12: ").strip()
        if text.lower() in ("?", "h", "help"):
            print("[HELP] Введите порты через пробел/запятую/точку с запятой.", flush=True)
            print("[HELP] Примеры: COM10 COM11 COM12 | 10,11,12 | COM10;COM12", flush=True)
            print("[HELP] Можно указать pyserial URL, например: socket://127.0.0.1:7001", flush=True)
            continue
        parsed = parse_ports_text(text)
        if parsed:
            return parsed
        print("[WARN] Список портов пустой. Введите хотя бы один порт.", flush=True)


def ask_enable_sealer_emulation() -> bool:
    while True:
        try:
            answer = input("Запустить имитацию запайщика на COM10? (да/нет): ").strip().lower()
        except EOFError:
            print("[WARN] Ввод недоступен, имитация запайщика выключена.", flush=True)
            return False
        except KeyboardInterrupt:
            print("\n[INFO] Вопрос по имитации прерван, имитация выключена.", flush=True)
            return False
        if answer in ("?", "h", "help"):
            print("[HELP] да/y  -> включить SEAL EMU AUTO ON на COM10", flush=True)
            print("[HELP] нет/n -> не включать (Enter тоже = нет)", flush=True)
            continue
        if answer in ("да", "д", "yes", "y"):
            return True
        if answer in ("нет", "н", "no", "n", ""):
            return False
        print("[WARN] Введите 'да' или 'нет'.", flush=True)


def print_startup_command_reference(effective_scenario_mode: bool) -> None:
    print("[INFO] Команды и режимы serial_multi_capture:", flush=True)
    print("[INFO]   --help", flush=True)
    print("[INFO]      Полный список CLI-опций запуска.", flush=True)
    print("[INFO]   Простой режим (без --scenario-mode)", flush=True)
    print("[INFO]      Логирование портов + остановка по клавише/Ctrl+C.", flush=True)
    print("[INFO]   Scenario mode (--scenario-mode, --scenario, --command-script)", flush=True)
    print("[INFO]      Включает runtime host-команды bench orchestrator.", flush=True)
    print("[INFO] Runtime host-команды:", flush=True)
    for cmd, description in RUNTIME_HELP_ITEMS:
        print(f"[INFO]   {cmd}", flush=True)
        print(f"[INFO]      {description}", flush=True)
    print(
        "[INFO]      scenario standard defaults: "
        f"auto_pause_after_s={int(STANDARD_SCENARIO_PAUSE_AFTER_S)}; "
        f"auto_stop_after_s={int(STANDARD_SCENARIO_STOP_AFTER_S)}",
        flush=True,
    )
    print("[INFO]      role aliases: conv|conveyor, manip|manipulator, master, COMx.", flush=True)
    if not effective_scenario_mode:
        print("[INFO] Подсказка: runtime host-команды доступны в interactive host> только с --scenario-mode.", flush=True)


def resolve_scenario_value(default_value, override_value):
    if override_value is None:
        return default_value
    return override_value


def validate_positive_optional(name: str, value: Optional[float]) -> None:
    if value is None:
        return
    if value <= 0:
        raise ValueError(f"{name} must be > 0")


def main() -> int:
    args = parse_args()
    role_ports = normalize_role_ports(args)
    scenario_name = args.scenario.lower() if args.scenario else None
    effective_scenario_mode = args.scenario_mode or scenario_name is not None or args.command_script is not None
    print_startup_command_reference(effective_scenario_mode)

    try:
        validate_positive_optional("--auto-common-pause-after-s", args.auto_common_pause_after_s)
        validate_positive_optional("--auto-stop-after-s", args.auto_stop_after_s)
        validate_positive_optional("--auto-stop-on-idle-after-s", args.auto_stop_on_idle_after_s)
    except ValueError as exc:
        print(f"[ERROR] {exc}", flush=True)
        return 2

    auto_stop_on_idle_enabled = args.auto_stop_on_idle or (args.auto_stop_on_idle_after_s is not None)

    if effective_scenario_mode:
        raw_ports = args.ports if args.ports else list(role_ports.values())
        selected_ports = normalize_ports(raw_ports)
        for role_port in role_ports.values():
            if role_port not in selected_ports:
                selected_ports.append(role_port)
        enable_seal_emu = False
    else:
        user_ports = prompt_ports_if_needed(args.ports)
        selected_ports = normalize_ports(user_ports)
        enable_seal_emu = ask_enable_sealer_emulation()

    capture = MultiSerialCapture(
        ports=selected_ports,
        baudrate=args.baud,
        output_dir=args.output_dir,
        session_prefix=args.session_prefix,
        read_timeout_s=args.read_timeout_s,
        duration_s=args.duration_s,
        role_ports=role_ports,
        idle_timeout_s=args.idle_timeout_s,
        reset_pulse_s=args.reset_pulse_s,
        reset_settle_s=args.reset_settle_s,
        auto_stop_on_idle=auto_stop_on_idle_enabled,
        auto_stop_on_idle_after_s=args.auto_stop_on_idle_after_s,
    )

    start_rc = capture.start()
    if start_rc != 0:
        return start_rc

    if effective_scenario_mode:
        capture._console(
            "[INFO] Роли: "
            f"conveyor={role_ports[ROLE_CONVEYOR]}, "
            f"manipulator={role_ports[ROLE_MANIPULATOR]}, "
            f"master={role_ports[ROLE_MASTER]}"
        )
        capture._print_runtime_help()

    if enable_seal_emu:
        capture.enable_terminal_echo_for_port(role_ports[ROLE_CONVEYOR], duration_s=6.0)
        capture.send_command(role_ports[ROLE_CONVEYOR], "SEAL EMU AUTO ON")

    try:
        if effective_scenario_mode:
            if scenario_name == "prep":
                capture.run_scenario_prep(
                    seal_emu_auto_on=args.seal_emu_auto_on or True,
                    plate_profile=resolve_scenario_value("generic", args.plate_profile),
                    start_on_sensor=args.start_on_sensor,
                    auto_common_start=args.auto_common_start,
                    auto_common_pause_after_s=resolve_scenario_value(None, args.auto_common_pause_after_s),
                    auto_stop_after_s=resolve_scenario_value(None, args.auto_stop_after_s),
                )
            elif scenario_name == "standard":
                capture.run_scenario_standard(
                    seal_emu_auto_on=args.seal_emu_auto_on or True,
                    plate_profile=resolve_scenario_value("realistic", args.plate_profile),
                    start_on_sensor=args.start_on_sensor or True,
                    auto_common_start=args.auto_common_start or True,
                    auto_common_pause_after_s=resolve_scenario_value(
                        STANDARD_SCENARIO_PAUSE_AFTER_S,
                        args.auto_common_pause_after_s,
                    ),
                    auto_stop_after_s=resolve_scenario_value(
                        STANDARD_SCENARIO_STOP_AFTER_S,
                        args.auto_stop_after_s,
                    ),
                )

            if args.command_script is not None:
                capture.run_command_script(args.command_script)

            if args.scenario_mode and sys.stdin.isatty():
                capture.run_command_console()
            else:
                if not sys.stdin.isatty():
                    capture._console("[INFO] stdin non-interactive, running without host console.")
                capture.wait(allow_keypress_stop=False)
        else:
            capture.wait(allow_keypress_stop=True)
    except KeyboardInterrupt:
        capture._console("[INFO] Получен Ctrl+C, останавливаю сессию...")
    finally:
        capture.stop()
        capture._console(f"[INFO] Логи сохранены в: {capture.output_dir.resolve()}")
        if capture.trace_file_path is not None:
            capture._console(f"[INFO] Session trace сохранен: {capture.trace_file_path.resolve()}")
        capture._console("[INFO] Сессия завершена.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
