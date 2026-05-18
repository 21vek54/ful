# master_control_com12: minimal / non-production

Этот каталог сохранен как минимальный экспериментальный мастер.

Для `version_5` production-прошивка `COM12` берется из:
- `archive/firmware/master_control_com12_legacy_2026-03-25`

Защита от случайной прошивки:
- в `platformio.ini` подключен `scripts/guard_non_production_upload.py`;
- `pio run -t upload` из этого каталога блокируется по умолчанию.

Осознанный override для эксперимента:
- `set ALLOW_MINIMAL_COM12_UPLOAD=1`
- затем запуск `pio run -t upload` в этом каталоге.
