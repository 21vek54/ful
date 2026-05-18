# Agent FULL Repair Loop

Скрипт: `tools/service/agent_ful_repair_loop.py`
Обертка: `tools/service/run_agent_ful_repair_loop.cmd`

Назначение: автоматический внешний цикл для `ful`-среды:

- модель делает минимальный patch;
- loop применяет patch;
- loop прошивает платы;
- loop запускает стендовый прогон;
- loop возвращает результат модели;
- модель итеративно продолжает до `AUTO_DONE`.

## Протокол ответа модели (строго)

Только один блок действия за ход:

1. `AUTO_APPLY_DIFF:` + fenced unified diff
2. `AUTO_FLASH:` + `com10|com11|com12|all`
3. `AUTO_RUN:` + команда `python tools/service/serial_multi_capture.py ...`
4. `AUTO_DONE:` + одна строка summary

## Быстрый старт

Создай файл цели, например `C:\temp\ful_repair_goal.txt`, и запусти:

```powershell
tools\service\run_agent_ful_repair_loop.cmd C:\temp\ful_repair_goal.txt
```

Готовый шаблон цели:

`tools/service/ful_repair_goal.example.txt`

## Ограничения безопасности

- `AUTO_APPLY_DIFF` разрешен только в путях:
  - `firmware/`
  - `archive/firmware/`
  - `tools/service/`
  - `docs/`
  - `Plan/`
  - `bin/`
- `AUTO_RUN` разрешает только запуск `python ... serial_multi_capture.py`.
- `AUTO_FLASH` делает только `pio run -t upload` в production-каталогах COM10/11/12.
- есть лимит шагов и стоп при многократном повторе одной и той же ошибки.

## Где смотреть артефакты

`logs/agent_ful_repair_loop/<timestamp>/`

Внутри:
- `session_meta.json`
- `instructions.txt`
- `goal.txt`
- `step_XX_response.json`
- `step_XX_assistant.txt`
- `step_XX_result.json`
- `step_XX.diff` (если был `AUTO_APPLY_DIFF`)
