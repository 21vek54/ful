# Контракт POST7 STOP И Recovery

Дата: 2026-04-06

### Назначение

Этот файл фиксирует архитектурный контракт для:

- `POST7 STOP`
- `manual recovery required`
- `POST7 RECOVERY CLEAR`

Документ нужен, чтобы не смешивать:

- факт, что `SEAL` уже когда-то был commit-нут;
- факт, что `SEAL` все еще реально находится в опасной зоне;
- отдельную post-seal фазу, где `SEAL` уже завершен, но `OUTFEED` еще не доведен до `ready_for_batch`.

## Граница ответственности

- `MASTER` и `COMMON` не владеют внутренней механикой `SEALER + OUTFEED`.
- `MASTER` работает только через high-level контракт `POST7 START LOAD`, `POST7 START UNLOAD_ONLY`, `POST7 STOP`.
- `COM10` обязан локально различать coarse-фазы `POST7`, потому что именно он владеет:
  - состоянием `SEALER`
  - состоянием `OUTFEED`
  - моментом, где stop еще unsafe
  - моментом, где `SEAL` уже завершен, а дальше остался только хвост отвода

Новый межплатный контракт для этого решения не требуется.

## Coarse-фазы POST7

Минимальная правильная модель для `POST7 LOAD`:

- `pre-seal`
  - `POST7 START LOAD` уже принят
  - `OUTFEED` уже может быть запущен
  - commit в `SEAL` еще не достигнут
  - stop здесь разрешен без recovery

- `seal-in-flight`
  - локальный старт `SEAL` уже принят
  - цикл запайки для текущего блока еще не подтвержден как завершенный
  - это единственная recovery-zone

- `seal-completed-outfeed-pending`
  - текущий цикл `SEAL` уже завершен локально
  - новый блок в `SEALER` уже считается завершенным
  - `POST7` еще active только потому, что `OUTFEED` не дошел до `outfeed_ready_for_batch = 1`
  - stop здесь уже не должен поднимать `manual recovery required`

Для `POST7 START UNLOAD_ONLY` используется только outfeed/post-step7 часть без новой запайки.

## Граница safe-stop

`manual recovery required` должен подниматься не после любого `seal commit`, а только пока `SEAL` реально не доведен до локально подтвержденного завершения.

Базовая граница:

- unsafe-зона начинается после успешного локального старта `SEAL`
- unsafe-зона заканчивается, когда текущий `sealer_completion_seq` уже продвинулся относительно `seq_base`

Если по механике выяснится, что `completion_seq` приходит чуть раньше реальной безопасной точки, `COM10` может локально добавить небольшой settle-хвост после completion.
Но даже в этом случае recovery-зона все равно определяется локально на `COM10`, а не переносится в `MASTER`.

## Что Значит POST7 STOP

`POST7 STOP` в этом контексте означает не аварийный стоп всей машины, а abort текущей локальной задачи post-step7 на `COM10`.

То есть `POST7 STOP` должен:

- прекратить текущий `POST7` job;
- остановить локальный `OUTFEED/OTCYCLE`;
- не фейковать production-success, если job не был доведен до штатного конца;
- поднять `manual recovery required` только если stop попал в recovery-zone `seal-in-flight`.

## Поведение POST7 STOP По Зонам

### `pre-seal`

Что это значит:

- новые `6` тарелок уже относятся к текущему `POST7 LOAD`
- но локальный `SEAL` еще не ушел в необратимую фазу

Что делает `POST7 STOP`:

- abort `POST7`
- abort `OUTFEED/OTCYCLE`
- `manual recovery required` не поднимается

### `seal-in-flight`

Что это значит:

- текущий блок из `6` тарелок в `SEALER` еще находится в процессе запайки

Что делает `POST7 STOP`:

- abort `POST7`
- abort `OUTFEED/OTCYCLE`
- поднимает `manual recovery required`

### `seal-completed-outfeed-pending`

Что это значит:

- текущий блок из `6` тарелок в `SEALER` уже запаян
- `POST7` еще не завершен только из-за отвода
- старый готовый блок еще может находиться на `OUT2 / MAIN_OUT`

Что делает `POST7 STOP`:

- abort `POST7`
- abort `OUTFEED/OTCYCLE`
- `manual recovery required` не поднимается
- production-success не объявляется

Это состояние является `safe stop`, но не является `job completed`.

## Что Должен Видеть MASTER

`MASTER` не должен различать внутренние фазы `pre-seal`, `seal-in-flight`, `seal-completed-outfeed-pending`.

Для `MASTER` сохраняется существующий coarse-контракт:

- `POST7 LOAD` completed:
  - `outfeed_ready_for_batch = 1`
  - `sealer_completion_seq != seq_base`

- `POST7 UNLOAD_ONLY` completed:
  - `outfeed_ready_for_batch = 1`

- fault/interlock:
  - существующий `alarm/errorWord`
  - `manual recovery required` как production-visible interlock

## Контракт Для POST7 RECOVERY CLEAR

`POST7 RECOVERY CLEAR` остается доверенной сервисной командой оператора.

Ее смысл:

- оператор вручную проверил механику;
- оператор вручную убрал или довел проблемное состояние;
- после этого оператор разрешает снять latch `manual recovery required`.

`POST7 RECOVERY CLEAR` не является автоматическим доказательством, что система сама аппаратно подтвердила полный safe-state.

## Минимальные Guard Для POST7 RECOVERY CLEAR

Оправданы только минимальные guards:

- `CLEAR` запрещен, пока `POST7` еще active
- `CLEAR` запрещен, пока `SEAL` все еще реально in-flight

Не требуется:

- раздувать `I2C`-контракт новыми битами только ради `CLEAR`
- переносить в `MASTER` знание о локальном механическом safe-clear
- превращать `CLEAR` в сложную полуавтоматическую recovery-процедуру

## Где Должны Быть Более Строгие Проверки

Более строгие проверки нужны не в `POST7 RECOVERY CLEAR`, а в следующем production-start.

То есть:

- `POST7 RECOVERY CLEAR` означает: оператор подтвердил ручное восстановление
- следующий `POST7 START LOAD` означает: прошивка еще раз проверила, что локальные стартовые условия действительно допустимы

Это сохраняет правильную границу ответственности:

- `CLEAR` остается сервисной операторской командой
- production-start остается проверяемым машиной действием

## Итоговое Решение

Для `version_4` принимается следующая модель:

- `manual recovery required` поднимается только внутри `seal-in-flight`
- после `sealer_completion_seq advanced`, но до `outfeed_ready_for_batch = 1`, состояние считается `seal-completed-outfeed-pending`
- `POST7 STOP` в этой post-seal зоне допустим без recovery latch
- `POST7 RECOVERY CLEAR` остается доверенной сервисной командой с минимальными guard-ограничениями

Эта модель не ломает:

- `COMMON -> POST7`
- `outfeed_ready_for_batch`
- `manual recovery` как production-visible interlock
- уже принятый coarse-контракт для `MASTER`
