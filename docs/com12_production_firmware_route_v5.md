# COM12: production маршрут прошивки (version_5)

Дата фиксации: `2026-04-08`

## Канонический источник прошивки COM12

- `archive/firmware/master_control_com12_legacy_2026-03-25`

Именно этот каталог считается production-актуальным для `version_5`.

## Что не считается production

- `firmware/master_control_com12`

Этот проект сохранен как `minimal/non-production` для экспериментов и чтения.
По умолчанию upload из него заблокирован guard-скриптом.

## Штатные helper-маршруты

- `tools/flash/flash_master_com12.cmd` -> production-каталог
- `bin/Прошить мастер COM12.cmd` -> production-каталог
- `bin/Монитор мастер COM12.cmd` -> production-каталог
- `tools/service/serial_multi_capture.py` (шаг прошивки COM12) -> production-каталог

## Осознанный запуск minimal-прошивки (только вручную)

Если нужен эксперимент с minimal-прошивкой:
1. открыть `firmware/master_control_com12`
2. явно установить `ALLOW_MINIMAL_COM12_UPLOAD=1`
3. только после этого запускать `pio run -t upload`
