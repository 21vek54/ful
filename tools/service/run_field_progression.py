import threading
import time
from datetime import datetime
from pathlib import Path

import serial


PORT_CONFIG = {
    "COM10": {"baud": 115200},
    "COM11": {"baud": 115200},
    "COM12": {"baud": 115200},
}


def now_iso() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]


class MultiPortLogger:
    def __init__(self, log_dir: Path):
        self.log_dir = log_dir
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.combined_path = self.log_dir / "combined.log"
        self.command_path = self.log_dir / "commands.log"
        self.ports = {}
        self.port_files = {}
        self.reader_threads = []
        self.stop_event = threading.Event()
        self.lock = threading.Lock()

        self.combined_file = self.combined_path.open("w", encoding="utf-8", newline="\n")
        self.command_file = self.command_path.open("w", encoding="utf-8", newline="\n")

    def open_ports(self):
        for port, cfg in PORT_CONFIG.items():
            ser = serial.Serial(
                port=port,
                baudrate=cfg["baud"],
                timeout=0.2,
                write_timeout=1.0,
            )
            # Ensure DTR is asserted after open.
            ser.dtr = True
            self.ports[port] = ser
            self.port_files[port] = (self.log_dir / f"{port}.log").open("w", encoding="utf-8", newline="\n")
            self.mark(f"PORT {port} opened at {cfg['baud']} baud")

    def start_readers(self):
        for port, ser in self.ports.items():
            t = threading.Thread(target=self._reader, args=(port, ser), daemon=True)
            t.start()
            self.reader_threads.append(t)

    def _reader(self, port: str, ser: serial.Serial):
        while not self.stop_event.is_set():
            try:
                raw = ser.readline()
            except Exception as exc:
                self.mark(f"READ ERROR {port}: {exc}")
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            ts = now_iso()
            row = f"{ts} [{port}] {line}"
            with self.lock:
                self.combined_file.write(row + "\n")
                self.combined_file.flush()
                self.port_files[port].write(f"{ts} {line}\n")
                self.port_files[port].flush()

    def mark(self, text: str):
        ts = now_iso()
        row = f"{ts} [MARK] {text}"
        with self.lock:
            self.combined_file.write(row + "\n")
            self.combined_file.flush()

    def send(self, port: str, cmd: str):
        ts = now_iso()
        row = f"{ts} -> {port}: {cmd}"
        with self.lock:
            self.command_file.write(row + "\n")
            self.command_file.flush()
            self.combined_file.write(f"{ts} [CMD ] {port} {cmd}\n")
            self.combined_file.flush()

        ser = self.ports[port]
        payload = (cmd + "\n").encode("utf-8")
        ser.write(payload)
        ser.flush()

    def close(self):
        self.stop_event.set()
        time.sleep(0.4)
        for ser in self.ports.values():
            try:
                ser.close()
            except Exception:
                pass
        for f in self.port_files.values():
            f.close()
        self.combined_file.close()
        self.command_file.close()


def send_required_status(logger: MultiPortLogger, label: str):
    logger.mark(f"STATUS SNAPSHOT {label}")

    logger.send("COM12", "STATE")
    time.sleep(0.35)
    logger.send("COM12", "COMMON STATUS")
    time.sleep(0.35)
    logger.send("COM12", "MBSCAN STATUS")
    time.sleep(0.35)

    logger.send("COM10", "SEAL STATUS")
    time.sleep(0.35)
    logger.send("COM10", "OTCYCLE STATUS")
    time.sleep(0.35)


def run_window(logger: MultiPortLogger, seconds: int, poll_every: int, label: str):
    start = time.monotonic()
    next_poll = 0
    poll_index = 0
    while True:
        elapsed = int(time.monotonic() - start)
        if elapsed >= seconds:
            return
        if elapsed >= next_poll:
            poll_index += 1
            send_required_status(logger, f"{label}_poll{poll_index:02d}")
            next_poll += poll_every
        time.sleep(0.5)


def run_a(logger: MultiPortLogger):
    logger.mark("RUN A START (MBSCAN ON)")
    logger.send("COM12", "MBSCAN ON")
    time.sleep(1.5)
    logger.send("COM12", "COMMON START")
    run_window(logger, seconds=90, poll_every=20, label="A")
    logger.mark("RUN A END")


def run_b(logger: MultiPortLogger):
    logger.mark("RUN B START (MBSCAN OFF)")
    logger.send("COM12", "MBSCAN OFF")
    time.sleep(1.5)
    logger.send("COM12", "COMMON START")
    run_window(logger, seconds=90, poll_every=20, label="B")
    logger.mark("RUN B END")


def run_c(logger: MultiPortLogger):
    logger.mark("RUN C START (MBSCAN ON -> OFF -> ON)")
    logger.send("COM12", "MBSCAN ON")
    time.sleep(1.5)
    logger.send("COM12", "COMMON START")

    total_seconds = 180
    off_at = 60
    on_at = 120
    poll_every = 20

    start = time.monotonic()
    next_poll = 0
    poll_index = 0
    sent_off = False
    sent_on = False

    while True:
        elapsed = int(time.monotonic() - start)
        if elapsed >= total_seconds:
            break

        if elapsed >= next_poll:
            poll_index += 1
            send_required_status(logger, f"C_poll{poll_index:02d}")
            next_poll += poll_every

        if (not sent_off) and elapsed >= off_at:
            logger.mark("RUN C TOGGLE -> MBSCAN OFF")
            logger.send("COM12", "MBSCAN OFF")
            sent_off = True

        if (not sent_on) and elapsed >= on_at:
            logger.mark("RUN C TOGGLE -> MBSCAN ON")
            logger.send("COM12", "MBSCAN ON")
            sent_on = True

        time.sleep(0.5)

    logger.mark("RUN C END")


def main():
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = Path("logs") / "field_runs" / f"{stamp}_com12_legacy_mbswitch"

    logger = MultiPortLogger(log_dir)
    logger.open_ports()
    logger.start_readers()

    try:
        logger.mark("BOOT STABILIZE START")
        time.sleep(8)
        send_required_status(logger, "bootstrap")

        run_a(logger)
        run_b(logger)
        run_c(logger)

        send_required_status(logger, "final")
        logger.mark("FIELD RUN COMPLETE")
        print(str(log_dir))
    finally:
        logger.close()


if __name__ == "__main__":
    main()
