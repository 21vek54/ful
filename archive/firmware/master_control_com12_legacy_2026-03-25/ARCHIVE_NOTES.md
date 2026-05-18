# Legacy master COM12 (production для version_5)

Дата исходной архивации: 2026-03-25

## Назначение

Этот каталог был создан как архив прежнего кода `master_control_com12`.
Начиная с `version_5` этот код снова используется как production-источник прошивки `COM12`.

Канонический production-путь:
- `archive/firmware/master_control_com12_legacy_2026-03-25`

Minimal/non-production проект оставлен отдельно:
- `firmware/master_control_com12`

## Что извлечено для нового проекта

- `I2C` адрес conveyor: `12`
- `I2C` адрес manipulator: `13`
- `I2C SDA`: `21`
- `I2C SCL`: `22`
- `MQTT base topic`: `fyl/master_com12`

## Локальные секреты

Реальные локальные `Wi-Fi` и `MQTT` секреты должны храниться только в локальных файлах, исключенных из git.
