# JT-Zero / MonkeysStab — Raspberry Pi 5

## Контрольная версия

- Репозиторий: mikhail-monkeysbrain/pi4_stab
- Ветка: test/ov5647-csi-v1
- Рабочий commit: c6b5119
- Платформа: Raspberry Pi 5
- Каталог: /home/vio/Desktop/monkeysStab_pi4

## Аппаратная конфигурация

| Компонент | Подключение |
|---|---|
| Полётный контроллер | /dev/ttyAMA3 |
| Скорость UART FC | 460800 |
| TF-Luna | /dev/ttyAMA2 |
| Основная камера | CSI OV5647 |
| Альтернативная камера | USB OV9281 |

## Штатный запуск

sudo systemctl start jtzero-flight.service

## Остановка

sudo systemctl stop jtzero-flight.service

## Перезапуск

sudo systemctl restart jtzero-flight.service

## Проверка состояния

systemctl status jtzero-flight.service --no-pager -l

## Просмотр журнала

sudo journalctl -u jtzero-flight.service -b -n 100 --no-pager

## Ручной запуск

Сначала остановить службу:

sudo systemctl stop jtzero-flight.service

Затем:

cd /home/vio/Desktop/monkeysStab_pi4
bash ./startFlightRuntime.sh

Остановка ручного запуска: Ctrl+C.

Не запускать одновременно ручной runtime и systemd-службу.

## Сетевые интерфейсы

- Web UI: http://192.168.2.1:8080
- Mission Planner: UDPCl, 192.168.2.1:14550
- MAVLink TCP: 127.0.0.1:5760

## Переменные окружения

- MONKEYS_FC_UART=/dev/ttyAMA3
- MONKEYS_FC_BAUD=460800
- MONKEYS_LUNA=/dev/ttyAMA2
- MONKEYS_RAW_UNIFIED_PUBLISH=1
- MONKEYS_FAST_RESTART=1
- MONKEYS_CAMERA_SOURCE=csi
- MONKEYS_WEB_PORT=8080

## Структура запуска

jtzero-flight.service
  -> startFlightRuntime.sh
    -> scripts/run_web.sh
      -> tools/web_service.py

Web UI управляет запуском полётного runtime.

## Systemd

Основной файл:

/etc/systemd/system/jtzero-flight.service

Дополнительные настройки:

- jtzero-flight.service.d/luna.conf
- jtzero-flight.service.d/network.conf
- jtzero-flight.service.d/override.conf

Важно: luna.conf содержит MONKEYS_LUNA=/dev/ttyAMA3,
но startFlightRuntime.sh переопределяет его на /dev/ttyAMA2.

## MAVLink

Заголовки MAVLink C восстанавливаются скриптом:

bash scripts/bootstrap_dependencies.sh

Зафиксированный commit библиотеки:

04fffaab116486ffdf7501c37a3f7393eb7beffc

## Проверка процессов

pgrep -af 'monkeysstab_optical_flow|web_service.py|mavlink_wifi_router.py'

## Ограничения

- Не изменять WORKED5 без отдельного тестирования.
- Не изменять параметры FC при развёртывании.
- Не заменять рабочие калибровки камер.
- Не выполнять git reset --hard или git clean -fd
  в рабочем каталоге без предварительной проверки.
- Не считать чистый Git-клон полностью настроенной
  системой до восстановления systemd и зависимостей.
