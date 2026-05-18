# Agent Autorun OpenAI Loop

Скрипт: `tools/service/agent_autorun_openai_loop.py`
Обертка: `tools/service/run_agent_autorun_openai_loop.cmd`

Назначение: внешний автоматический цикл через OpenAI API.

Он делает:
1. Отправляет initial prompt в модель.
2. Получает ответ ассистента.
3. Прогоняет ответ через `agent_autorun_bridge.py`.
4. Если есть `AUTO_RUN`, запускает `serial_multi_capture.py`.
5. Формирует `AUTO_RUN_BRIDGE_RESULT_JSON` и отправляет как следующий user-turn.
6. Повторяет до `no_trigger` или лимита ходов.

## Секрет-файл (рекомендуется)

Локальный файл с ключом:

`tools/service/openai_api_key_local.txt`

Формат:

```text
OPENAI_API_KEY=sk-...
```

Можно взять за основу шаблон:

`tools/service/openai_api_key_local.txt.example`

## Быстрый старт

```powershell
tools\service\run_agent_autorun_openai_loop.cmd C:\temp\bench_prompt.txt
```

Или напрямую:

```powershell
python tools/service/agent_autorun_openai_loop.py --prompt-file C:\temp\bench_prompt.txt --model gpt-5 --max-turns 8
```

Резервный вариант (если ключ через env):

```powershell
$env:OPENAI_API_KEY="sk-..."
tools\service\run_agent_autorun_openai_loop.cmd C:\temp\bench_prompt.txt
```

## Где смотреть результаты

По умолчанию сессия пишется в:

`logs/agent_autorun_loop/<timestamp>/`

Там лежат:
- `session_meta.json`
- `instructions.txt`
- `turn_XX_response.json` (сырой ответ API)
- `turn_XX_assistant.txt`
- `turn_XX_bridge_result.json`
- `turn_XX_next_user_payload.txt`

## Важные условия

- Нужен API-ключ OpenAI в локальном файле или в `OPENAI_API_KEY`.
- Скрипт ведет отдельный API-диалог и не может напрямую писать в текущий UI-чат.
- Bridge выполняет только строго валидный `AUTO_RUN` для `serial_multi_capture.py`.
