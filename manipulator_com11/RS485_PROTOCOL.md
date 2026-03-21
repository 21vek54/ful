# RS485 Protocol (manipulator_com11)

Скорость и формат:
- UART2: `115200 8N1`
- GPIO: `RX=16`, `TX=17`, `DE/RE=13`
- Полудуплекс, окончание строки: `\n` (допускается `\r\n`)

## Формат запросов
`#<seq> <CMD> [args...]`

Примеры:
- `#1 PING`
- `#2 STATUS`
- `#3 RUN`
- `#4 MOVE LEFT`

Если `#<seq>` не передан, контроллер отвечает с `#0`.

## Формат ответов
- Успех: `#<seq> OK [key=value ...]`
- Ошибка: `#<seq> ERR <code> <text>`
- Событие: `#0 EVT <name> [key=value ...]`

## Реализованные команды
- `PING`
- `STATUS`
- `SENSORS`
- `RUN`
- `RECOVER`
- `CALIBRATE`
- `MOVE LEFT|RIGHT`
- `Z UP|DOWN`
- `GRIP OPEN|CLOSE`
- `TEST START <cycles>`
- `TEST STOP`
- `GREASE START`
- `GREASE STOP`
- `ESTOP`
- `DRIVER RESET`
- `SYS RESET`
- `EEPROM CLEAR`
- `HELP`

## Поля STATUS
`mode, job, busy, calibrated, moving, dir, alarm_driver, safe, travel_steps, travel_mm, steps_per_mm, work_saved, work_step_saved, interrupted, lim_left, lim_right, z_up, z_down, grip_open, grip_closed, conflict_limits, conflict_z, conflict_grip`

## Поля SENSORS
`lim_left, lim_right, z_up, z_down, grip_open, grip_closed, alarm_driver, conflict_limits, conflict_z, conflict_grip`

## Коды ошибок
- `1` BUSY
- `2` NOT_CALIBRATED
- `3` SAFETY_CONFLICT
- `4` DRIVER_ALARM
- `5` INVALID_ARG
- `6` NOT_ALLOWED
- `7` INTERNAL
- `8` TIMEOUT

## События
- `EVT ALARM_ON`
- `EVT ALARM_OFF`
- `EVT JOB from=<old> to=<new>`
