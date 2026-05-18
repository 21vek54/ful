# Итог этапа: low-level учет тарелок (`version_5`)

Дата фиксации: 2026-04-15

## Контекст
Закрыт узкий production-вопрос по учету тарелок без отдельной cleanup-фазы: устранено расхождение между terminal accounting мастера и фактической механикой линии за счет честной передачи/интерпретации `unknown`.

## Причины mismatch (до фикса)
1. На `COM10` неопределенные состояния схлопывались в ложный `0` на wire-уровне вместо `unknown`, из-за чего `COM12` мог принимать неверный terminal accounting.
   `firmware/conveyor_control_com10/src/app/pause_contract.cpp:319`

2. На `COM10` отсутствует отдельный достоверный датчик "sealer пуст", поэтому "нет признака остатка" нельзя трактовать как `0`; это `unknown`.
   `firmware/conveyor_control_com10/src/app/conveyor_status_runtime.cpp:107`

3. На `COM12` финальные pause-решения ранее недостаточно жестко разделяли trusted/unknown по счетчикам; теперь источник и доверенность учитываются явно.
   `archive/firmware/master_control_com12_legacy_2026-03-25/src/master_main.cpp:330`
   `archive/firmware/master_control_com12_legacy_2026-03-25/src/master_main.cpp:2328`

4. `COM11` не является источником plate-count accounting (только pause epoch/ack), поэтому его механика не должна "рисовать" остаток на `COM12`.
   `firmware/manipulator_com11/src/main_10.inc:365`
   `firmware/manipulator_com11/src/main_10.inc:623`

## Что изменено
1. На `COM10` добавлен явный low-level статус: `feedBufferCount/feedIn2Pairs/sealerPlateCount` + `known`-флаги.
   `firmware/conveyor_control_com10/src/app/conveyor_status.h:22`

2. `COM10 runtime` теперь передает `unknown` честно; для sealer не подставляется искусственный `0` при недостоверности.
   `firmware/conveyor_control_com10/src/app/conveyor_status_runtime.cpp:100`
   `firmware/conveyor_control_com10/src/app/conveyor_status_runtime.cpp:122`

3. В pause wire-контракте `COM10` введен sentinel `0xF` для `UNKNOWN`; добавлены диагностические `PAUSE STATUS` логи (local/wire/known).
   `firmware/conveyor_control_com10/src/app/pause_contract.cpp:18`
   `firmware/conveyor_control_com10/src/app/pause_contract.cpp:398`

4. На `COM12` обновлен ingest/decision path: `0xF -> unknown`, `PauseCountSource`, более строгие finalize-gates, source-aware причины и MQTT/serial диагностика.
   `archive/firmware/master_control_com12_legacy_2026-03-25/src/master_main.cpp:1206`
   `archive/firmware/master_control_com12_legacy_2026-03-25/src/master_main.cpp:2802`
   `archive/firmware/master_control_com12_legacy_2026-03-25/src/master_main.cpp:5662`

## Что проверено
1. Успешные сборки (2026-04-15):
   - `firmware/conveyor_control_com10`
   - `firmware/manipulator_com11`
   - `archive/firmware/master_control_com12_legacy_2026-03-25`

2. До фикса mismatch подтвержден логом:
   `logs/serial/realplates_terminal_20260409_204111_COM12.txt:272` (`buffer_count=1, sealer_count=0` в terminal-эскалации).

3. После фикса `COM10` отдает `unknown`, а не ложный `0`:
   `logs/serial/realplates_lowlevelfix_pause_20260415_133341_COM10.txt:108`

4. После фикса `COM12` не трактует это как trusted-terminal:
   `logs/serial/realplates_lowlevelfix_pause_20260415_133341_COM12.txt:92`
   `logs/serial/realplates_lowlevelfix_pause_20260415_133341_COM12.txt:100`

5. Повторный realplates-ретест после устранения внешней помехи манипулятору (2026-04-15):
   - `PREP INIT` корректен: `logs/serial/realplates_retest_after_clear_20260415_135059_COM11.txt:70`
   - terminal outcome снова честный `manual_recovery_required`, с `sealer_source=wire_unknown`, без фальшивого `sealer=0`:
     `logs/serial/realplates_retest_after_clear_20260415_135059_COM12.txt:1609`
     `logs/serial/realplates_retest_after_clear_20260415_135059_COM12.txt:1610`

## Статус `pause_empty`
`pause_empty` пока не подтвержден как архитектурно достижимый в текущем железном контракте.

При этом требование "100% честный terminal accounting без расхождения с механикой" выполнено: при недостоверности остатка система фиксирует `unknown` и не принимает финальное решение на схлопнутых данных.
