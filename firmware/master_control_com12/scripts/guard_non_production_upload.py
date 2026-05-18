"""Guard accidental upload from non-production COM12 project."""

import os

Import("env")

targets = [str(target).lower() for target in COMMAND_LINE_TARGETS]
is_upload = any("upload" in target or target == "program" for target in targets)

if is_upload:
    allow_upload = os.environ.get("ALLOW_MINIMAL_COM12_UPLOAD", "").strip().lower()
    if allow_upload not in ("1", "true", "yes"):
        print("")
        print("[BLOCKED] firmware/master_control_com12 помечен как minimal/non-production.")
        print("[BLOCKED] Штатный production-маршрут COM12:")
        print("          archive/firmware/master_control_com12_legacy_2026-03-25")
        print("[BLOCKED] Для осознанного теста minimal установите:")
        print("          ALLOW_MINIMAL_COM12_UPLOAD=1")
        print("")
        env.Exit(1)
