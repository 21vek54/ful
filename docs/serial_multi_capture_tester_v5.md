# Тестер `serial_multi_capture.py` Для `version_5`

Дата фиксации: `2026-04-09`

## Назначение

`tools/service/serial_multi_capture.py` — это стендовый orchestrator для:

- одновременного снятия логов с `COM10`, `COM11`, `COM12`;
- reset плат;
- сервисного запуска `COMMON START` / `COMMON PAUSE`;
- включения сервисной эмуляции:
  - `SEAL EMU AUTO ON`
  - `INFEED EMU`;
- сценарных прогонов `prep` и `standard`;
- автоматического завершения capture по timeout, idle или terminal outcome.

Этот документ описывает, как пользоваться тестером в `version_5`.

## Что Нужно Прочитать Перед Запуском

Перед работой с тестером нужно держать в голове два документа:

- [PLSN.md](/C:/Users/Пользователь/Documents/PlatformIO/Projects/fyl/PLSN.md)
- [com12_production_firmware_route_v5.md](/C:/Users/Пользователь/Documents/PlatformIO/Projects/fyl/docs/com12_production_firmware_route_v5.md)

Особенно важно:

- production master для `COM12` в `version_5` — это legacy-прошивка;
- случайно запускать тест с `minimal/non-production` master нельзя.

## Роли И Порты По Умолчанию

- `COM10` = `conveyor`
- `COM11` = `manipulator`
- `COM12` = `master`

Если порты на стенде другие, их нужно явно передать через:

- `--conveyor-port`
- `--manipulator-port`
- `--master-port`

## Куда Пишутся Логи

По умолчанию тестер пишет все в:

- `logs/serial`

На одну сессию создаются:

- лог `COM10`
- лог `COM11`
- лог `COM12`
- `session_trace`

Имя сессии выглядит так:

- `<session-prefix>_YYYYMMDD_HHMMSS_COM10.txt`
- `<session-prefix>_YYYYMMDD_HHMMSS_COM11.txt`
- `<session-prefix>_YYYYMMDD_HHMMSS_COM12.txt`
- `<session-prefix>_YYYYMMDD_HHMMSS_session_trace.txt`

Если `--session-prefix` не задан, префикс будет пустым.

## Основные Режимы Работы

### 1. Простое Снятие Логов

Только открыть порты и писать лог, без сценария:

```powershell
python tools/service/serial_multi_capture.py COM10 COM11 COM12
```

Или с ролями по умолчанию:

```powershell
python tools/service/serial_multi_capture.py --conveyor-port COM10 --manipulator-port COM11 --master-port COM12
```

### 2. `scenario prep`

Сценарий подготовки:

- reset плат;
- опциональный `SEAL EMU AUTO ON`;
- опциональный `INFEED EMU`;
- без обязательного `COMMON START`.

Пример:

```powershell
python tools/service/serial_multi_capture.py --session-prefix prep_check --scenario prep --seal-emu-auto-on --plate-profile realistic --start-on-sensor --conveyor-port COM10 --manipulator-port COM11 --master-port COM12
```

### 3. `scenario standard`

Основной стендовый сценарий `version_5`.

Он делает:

1. reset всех плат;
2. readiness wait после reset;
3. проверку начального состояния `COM11`;
4. если надо, сервисный `PREP INIT` на `COM11`;
5. опциональный `SEAL EMU AUTO ON`;
6. опциональный `plate profile`;
7. `COMMON START`;
8. scheduler:
   - auto `COMMON PAUSE`
   - auto-stop
   - outcome wait после pause.

### 4. Runtime Host-Консоль

Если включен `scenario-mode`, доступны host-команды:

- `help`
- `stop`
- `reset`
- `seal-on`
- `plate-pass-on`
- `plate-pass-off`
- `plate-pass-status`
- `common-start`
- `common-pause`
- `wait-idle`
- `scenario prep`
- `scenario standard`
- `send <role|port> <payload>`

## Рекомендуемые Рабочие Команды

### Synthetic / Виртуальные Тарелки

Быстрый synthetic-прогон с realistic-подачей:

```powershell
python tools/service/serial_multi_capture.py --session-prefix synth_pause --scenario standard --seal-emu-auto-on --plate-profile realistic --start-on-sensor --auto-common-pause-after-s 60 --auto-stop-after-s 240 --auto-stop-on-idle --auto-stop-on-idle-after-s 30 --idle-timeout-s 30 --conveyor-port COM10 --manipulator-port COM11 --master-port COM12
```

Когда использовать:

- проверка pause-контракта;
- проверка логики `COM12`;
- быстрые регрессионные прогоны без реальной подачи тарелок.

### Реальные Тарелки

Прогон без `INFEED EMU`:

```powershell
python tools/service/serial_multi_capture.py --session-prefix realplates --scenario standard --seal-emu-auto-on --plate-profile none --auto-common-pause-after-s 60 --auto-stop-after-s 240 --auto-stop-on-idle --auto-stop-on-idle-after-s 30 --idle-timeout-s 30 --conveyor-port COM10 --manipulator-port COM11 --master-port COM12
```

Когда использовать:

- проверка реальной механики;
- проверка `COMMON PAUSE` на физическом остатке тарелок;
- сверка логов с фактическим состоянием линии.

### Только Логирование Без Авто-Паузы

Если нужно просто снять цикл:

```powershell
python tools/service/serial_multi_capture.py --session-prefix capture_only --scenario standard --seal-emu-auto-on --plate-profile none --auto-stop-after-s 240 --conveyor-port COM10 --manipulator-port COM11 --master-port COM12
```

Если `--auto-common-pause-after-s` не задан, для `scenario standard` по умолчанию используется `60` секунд.

## Что Делает `scenario standard` Важного

### Readiness После Reset

Сценарий подтверждает готовность только после наблюдаемых markers:

- `COM10`: `I2C status: addr 12`
- `COM11`: `I2C status ready: addr 13`
- `COM12`:
  - console-ready после нового boot banner
  - post-banner `I2C-ready`

Это нужно, чтобы не стартовать `COMMON START` по stale heartbeat старого runtime.

### `COM11 PREP INIT`

Если манипулятор после reset не в состоянии `право/верх/разжат`, сценарий:

- не шлет `COMMON START` сразу;
- делает `PREP INIT`;
- ждет явного успешного completion.

### Outcome Wait После `COMMON PAUSE`

После auto или host `COMMON PAUSE` тестер не должен просто завершаться по обычному idle.

Он переходит в отдельный режим ожидания terminal outcome от `COM12`:

- `pause_hold`
- `pause_empty`
- `manual_recovery_required`

Если outcome получен:

- запускается короткий post-outcome timeout;
- затем сессия корректно закрывается.

Если outcome не получен:

- срабатывает отдельный `pause outcome wait timeout`.

## Важные Оговорки

### 1. `COM10: P1 buffer: есть 2 тарелки` Не Всегда Означает Реальный Остаток

На старте `COM10` может печатать:

- `P1 buffer: есть 2 тарелки`

Этот print может идти из сохраненного software-флага `Preferences`, а не из живого датчика.

Поэтому:

- эту строку нельзя автоматически считать физическим доказательством остатка;
- для реальной механики всегда нужно сверять:
  - логи;
  - фактическую линию;
  - операторское наблюдение.

### 2. Synthetic И Real Plates Нельзя Смешивать

`--plate-profile realistic` полезен для регрессии pause/runtime,
но не заменяет реальный прогон с тарелками.

Разделяй сессии по префиксам:

- `synth_*`
- `realplates_*`

### 3. `SEAL EMU AUTO ON` Это Сервисная Эмуляция

Она полезна для стенда и логики `POST7/WaitDone`,
но не является доказательством реального поведения запайщика.

### 4. `manual_recovery_required` Может Быть Честным Исходом

Если линия не может автоматически достичь:

- `pause_hold`
- или `pause_empty`

тестер должен увидеть именно явный terminal outcome,
а не silent hang.

## Как Читать Успешность Прогона

### Для Synthetic

Минимально полезный хороший результат:

- нет раннего baseline-abort;
- `COMMON PAUSE` отправлена;
- outcome получен явно;
- нет silent wait до watchdog без причины.

### Для Real Plates

Минимально полезный хороший результат:

- outcome в логах совпадает с фактической механикой;
- если мастер говорит `pause_empty`, линия реально пустая;
- если мастер говорит `manual_recovery_required`, остаток и причина понятны оператору.

## Что Смотреть После Прогона

Сначала всегда смотреть:

- `session_trace`

Потом уже:

- `COM12` как главный orchestration log;
- `COM10` для `P1`, `POST7`, pause prepare/ready;
- `COM11` для `PREP INIT`, рабочего цикла и механических timeout.

## Текущий Практический Статус На `2026-04-09`

На текущем состоянии `version_5` tester уже подтверждает:

- `COM10` safe-boundary pause behavior;
- `COM11 PREP INIT`;
- pause outcome wait вместо грубого idle-stop;
- явный terminal outcome от `COM12` без silent hang.

Но production-цель `pause_empty` еще не закрыта:

- на реальных тарелках линия может приходить к честному
  `manual_recovery_required`;
- при этом физический остаток тарелок на линии еще нужно отдельно согласовать
  с residual accounting мастера.
