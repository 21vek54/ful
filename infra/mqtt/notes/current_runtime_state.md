## Текущее состояние runtime MQTT / Node-RED (master_com12 minimal)

Дата фиксации: 2026-03-25

### 1) Текущий operational mode службы Mosquitto

- Служба Windows `mosquitto` работает в штатном service-режиме:
  - `BINARY_PATH_NAME = "C:\Program Files\Mosquitto\mosquitto.exe" run`
  - `SERVICE_START_NAME = LocalSystem`
- Реально слушаемый порт службы: `1884`.
- Процесс, который держит порт: `mosquitto.exe` (PID проверяется через `sc queryex` / `netstat`).
- Текущий активный системный конфиг слушает:
  - `127.0.0.1:1884`
  - `192.168.1.136:1884`
- Широкий bind `0.0.0.0:1884` убран.

Проверка на 2026-03-25:
- `sc queryex mosquitto` -> `RUNNING`
- `Get-NetTCPConnection -State Listen -LocalPort 1884` -> `127.0.0.1:1884`, `192.168.1.136:1884`
- `netstat` -> `127.0.0.1:1884 LISTENING`, `192.168.1.136:1884 LISTENING`, без `0.0.0.0:1884`
- Входящие сообщения от `master_com12` на `status` и `online` принимаются через этот сервис.

### 2) Что было причиной ошибки 1053

При попытке запускать службу с `binPath` вида:
- `mosquitto.exe -c <project_conf>`

служба стабильно падала с таймаутом старта (`1053`), а в System log фиксировались события Service Control Manager `7000/7009`.

Практический вывод для этого ПК:
- для корректного старта именно как Windows-service нужно оставлять `binPath` в штатном виде `"...\mosquitto.exe" run`;
- переключение активного конфига нужно делать не через `binPath -c`, а через замену файла
  `C:\Program Files\Mosquitto\mosquitto.conf`.

### 3) Как теперь выполняется переключение 1883 / 1884

Сервисный скрипт:
- `tools/service/switch_mosquitto_service_to_project_conf.cmd`

Текущая логика скрипта:
1. Берёт исходный конфиг из репозитория:
   - `1884` (по умолчанию): `infra/mosquitto/conf/mosquitto_1884.conf`
   - `1883` (если передан аргумент `1883`): `infra/mosquitto/conf/mosquitto.conf`
2. Валидирует конфиг через `mosquitto.exe --test-config`.
3. Копирует его в системный путь:
   - `C:\Program Files\Mosquitto\mosquitto.conf`
4. Гарантирует штатный `binPath` службы:
   - `"...\mosquitto.exe" run`
5. Перезапускает службу с ожиданием `Stopped/Running`.
6. При ошибке выполняет rollback на резервную копию `mosquitto.conf`.

Практический вывод по расхождению предыдущих запусков:
- если скрипт запускается из non-elevated shell, он останавливается на precheck прав и не доходит до копирования файла;
- после запуска из реально elevated PowerShell ручная замена `C:\Program Files\Mosquitto\mosquitto.conf` прошла успешно, сервис стартовал с новым listener-конфигом.
- при включении `password_file` нужно отдельно проверить права чтения для `LocalSystem`:
  - если парольный файл создан с ACL только для пользователя, служба может уйти в `STOPPED` после старта;
  - рабочий вариант для этого ПК: парольный файл с доступом чтения для `NT AUTHORITY\SYSTEM`.

### 4) Состояние минимального контура master_com12

Топики:
- `fyl/master_com12/status`
- `fyl/master_com12/online`

Статус интеграции на 2026-03-25:
- `master_com12 <-> Mosquitto(service) <-> Node-RED` работает на порту `1884`.
- Примеры живых payload:
  - `status`: `{"wifi":true,"mqtt":true,"board12":"online","board13":"online"}`
  - `online`: `1`

### 5) Текущий уровень безопасности MQTT

- Текущий активный сервисный режим использует:
  - `listener 1884 127.0.0.1`
  - `listener_allow_anonymous false`
  - `listener 1884 192.168.1.136`
  - `listener_allow_anonymous false`
  - `password_file C:/ProgramData/Mosquitto/passwd/fyl_master_com12_1884.pw`
  - `acl_file C:/ProgramData/Mosquitto/acl/fyl_master_com12_1884.acl`
- `0.0.0.0` не используется, anonymous-доступ отключён.
- Учётки разведены по ролям:
  - `fyl_master_com12` (прошивка `master_control_com12`) имеет только publish-права:
    - `fyl/master_com12/status`
    - `fyl/master_com12/online`
  - `fyl_nodered_com12` (локальный `Node-RED`) имеет только read-права:
    - `fyl/master_com12/status`
    - `fyl/master_com12/online`
  - любые прочие авторизованные клиенты без отдельного ACL-правила не получают доступ к этим топикам.
- Локальный `Node-RED` сейчас работает через `127.0.0.1:1884`.
- Плата `master` сейчас работает через LAN-адрес ПК `192.168.1.136:1884`.
- Текущая локальная схема хранения настроек/секретов:
  - `C:\ProgramData\Mosquitto\passwd\fyl_master_com12_1884.pw` — парольный файл брокера (вне git);
  - `C:\ProgramData\Mosquitto\acl\fyl_master_com12_1884.acl` — ACL-файл брокера (runtime, вне git);
  - `firmware/master_control_com12/src/mqtt_secrets_local.h` — локальные credentials для `master` (файл исключён из git);
  - `infra/node-red/userdir/flows_cred.json` — runtime credentials `Node-RED` (файл не публиковать в git).
- Проверка:
  - анонимный клиент получает `Connection Refused: not authorised` (`mosquitto_sub` без `-u/-P`);
  - `master` продолжает публиковать `status/online`;
  - `Node-RED` продолжает читать `status/online`;
  - попытка `master` подписаться на `fyl/master_com12/status` не получает сообщений (`Timed out`, read-права не выданы ACL);
  - попытка `Node-RED` публиковать в `fyl/master_com12/status` отклоняется (`not authorised`).

### 6) Резервный режим

При необходимости можно быстро вернуть сервисный порт `1883`:
- `tools\service\switch_mosquitto_service_to_project_conf.cmd 1883`

И снова закрепить `1884`:
- `tools\service\switch_mosquitto_service_to_project_conf.cmd 1884`

### 7) Минимальный операторский web-интерфейс (Node-RED Dashboard)

Актуальный runtime:
- Node-RED стартует из project userDir:
  - `C:\Users\Пользователь\Documents\PlatformIO\Projects\fyl\infra\node-red\userdir`
- Dashboard base path:
  - `http://127.0.0.1:1880/dashboard`
- Прямая страница оператора для `master_com12`:
  - `http://127.0.0.1:1880/dashboard/master-com12`

Что отображается на странице:
- `board12`
- `board13`
- `wifi`
- `mqtt`
- `master online`
- `last update`

Нормализация значений в flow:
- `online`
- `offline`
- `unknown`

Проверка на 2026-03-25:
- HTTP-доступность:
  - `GET /dashboard` -> `200`
  - `GET /dashboard/master-com12` -> `200`
- Живой MQTT-трафик от master приходит в Node-RED:
  - `fyl/master_com12/status`
  - `fyl/master_com12/online`
- Обновление виджетов подтверждено через debug datastore Dashboard:
  - `/dashboard/_debug/datastore/ui_text_board12_minimal` -> `online`
  - `/dashboard/_debug/datastore/ui_text_board13_minimal` -> `online`
  - `/dashboard/_debug/datastore/ui_text_wifi_minimal` -> `online`
  - `/dashboard/_debug/datastore/ui_text_mqtt_minimal` -> `online`
  - `/dashboard/_debug/datastore/ui_text_master_online_minimal` -> `online`
  - `/dashboard/_debug/datastore/ui_text_last_update_minimal` -> актуальный timestamp
