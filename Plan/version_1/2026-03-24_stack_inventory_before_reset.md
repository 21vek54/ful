## Инвентаризация текущего стека перед сбросом

Дата: 2026-03-24

### Проект мастера

Путь:
`C:\Users\Пользователь\Documents\PlatformIO\Projects\fyl\master_control_com12`

Примечания:
- Это проект PlatformIO для ESP32-платы мастера.
- В корне проекта уже лежат выгрузки flow-файлов Node-RED:
  - `current_flows.json`
  - `node_red_master_flow.json`
  - `node_red_master_flow_ascii.json`

### Архив планов

Путь:
`C:\Users\Пользователь\Documents\PlatformIO\Projects\fyl\Plan\version_1`

Примечания:
- Существующие markdown-файлы планов были перенесены сюда из старого корня `Plan`.
- Эта папка считается замороженной опорной версией `version_1`.

### Node-RED

Путь:
`C:\Users\Пользователь\.node-red`

Основные файлы:
- `flows.json`
- `flows_cred.json`
- `settings.js`
- `.config.nodes.json`
- `.config.runtime.json`
- `.config.users.json`

Примечания:
- В этой папке уже есть много резервных копий flow-файлов.
- Команда запуска `node-red.cmd` установлена по пути:
  `C:\Users\Пользователь\AppData\Roaming\npm\node-red.cmd`
- Во время проверки Node-RED не был обнаружен как активный процесс или служба.

### MQTT-брокер

Основная установка:
`C:\Program Files\Mosquitto`

Служба Windows:
- Имя службы: `mosquitto`
- Отображаемое имя: `Mosquitto Broker`
- Тип запуска: `Auto`
- Состояние во время проверки: `Running`

Обнаруженный порт прослушивания:
- `127.0.0.1:1883`

Пользовательская папка MQTT:
`C:\Users\Пользователь\mqtt`

Найденный пользовательский конфиг:
- `mosquitto_1884.conf`

Важное примечание:
- Активная служба Windows сейчас слушает порт `1883`.
- Найденный в `C:\Users\Пользователь\mqtt` конфиг настроен на порт `1884`.
- Это означает, что запущенная служба Mosquitto, скорее всего, работает не по этому пользовательскому конфигу.
