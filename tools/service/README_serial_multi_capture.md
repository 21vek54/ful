# Serial Multi Capture

Простая CLI-утилита для одновременного снятия serial-логов с нескольких портов.

Скрипт: `tools/service/serial_multi_capture.py`

## Зависимость

```powershell
python -m pip install -r tools/service/requirements.txt
```

## Запуск

```powershell
python tools/service/serial_multi_capture.py COM10 COM11 COM12
```

Или просто без портов, тогда скрипт спросит их при старте (например, `10,12`):

```powershell
python tools/service/serial_multi_capture.py
```

С опциями:

```powershell
python tools/service/serial_multi_capture.py COM10 COM11 COM12 `
  --baud 115200 `
  --output-dir logs/serial `
  --session-prefix standA
```

## Что делает

- открывает порты параллельно;
- пишет каждый порт в отдельный `.txt` файл;
- формат строки: `[YYYY-MM-DD HH:MM:SS.mmm] <serial line>`;
- не падает полностью, если часть портов не открылась;
- корректно завершает сессию по любой клавише в терминале или по `Ctrl+C`.
- после выбора портов спрашивает, включать ли имитацию запайщика (`SEAL EMU AUTO ON` на `COM10`).
- если имитация включена, скрипт выводит в терминал ответы `COM10` в течение короткого окна после отправки команды.

## Полезная опция

```powershell
--duration-s 60
```

Автостоп через N секунд (для тестов). По умолчанию `0` и остановка только по `Ctrl+C`.
