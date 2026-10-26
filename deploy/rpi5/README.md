# JT-Zero — развёртывание на Raspberry Pi 5

## Контрольная версия

- ОС: Debian 13 (Trixie), aarch64
- Ветка: test/ov5647-csi-v1
- Рабочий код: c6b5119
- Конфигурация systemd и сети: 33513aa
- Каталог: /home/vio/Desktop/monkeysStab_pi4

## Подтверждённые системные зависимости

| Пакет | Версия на рабочей RPi5 |
|---|---|
| g++ | 4:14.2.0-1 |
| git | 1:2.47.3-0+deb13u1 |
| pkg-config | 1.8.1-4 |
| libopencv-dev | 4.10.0+dfsg-5 |
| libcamera-dev | 0.7.2+rpt20260817-1 |
| python3-opencv | 4.10.0+dfsg-5 |
| python3-numpy | 1:2.2.4+ds-1 |
| network-manager | 1.52.1-1+rpt4 |

Дополнительно на рабочей системе установлены:
- cmake
- libcamera-v4l2
- python3-serial

Их обязательность для основного runtime отдельно не подтверждена.

## Минимальная установка пакетов

Для Debian 13 с соответствующими репозиториями Raspberry Pi:

sudo apt update

sudo apt install -y \
  g++ git pkg-config \
  libopencv-dev libcamera-dev \
  python3-opencv python3-numpy \
  network-manager

Команды приведены для новой системы.
На работающей RPi5 повторно устанавливать пакеты не требуется.

## MAVLink

Основной runtime использует заголовки MAVLink C.

bash scripts/bootstrap_dependencies.sh

Зафиксированный commit MAVLink:

04fffaab116486ffdf7501c37a3f7393eb7beffc

## Сборка

bash scripts/smoke_build.sh

## Web UI

Реализован на стандартном Python ThreadingHTTPServer.

Flask не требуется.

## Системные службы

Сохранены в:

deploy/rpi5/systemd/

- jtzero-flight.service
- jtzero-network.service
- jtzero-flight.service.d/luna.conf
- jtzero-flight.service.d/network.conf
- jtzero-flight.service.d/override.conf

Сетевой скрипт:

deploy/rpi5/network/jtzero-network-select.sh

Эти файлы являются снимком рабочей конфигурации.

## NetworkManager

Необходимо создать профили:

- Dom — домашняя Wi-Fi-сеть
- JTZero-Flight — точка доступа

Для JTZero-Flight:
- SSID: JTZero-Flight
- Режим: AP
- Диапазон: 2,4 ГГц
- IPv4: shared
- Адрес: 192.168.2.1/24
- Автоподключение: выключено

Пароли и профили с секретами в Git не сохраняются.

## Важно

Это документация для восстановления, а не готовый
автоматический установщик.

Перед запуском на новой Raspberry Pi необходимо
проверить UART, настройки NetworkManager,
зависимости и доступность камер.

Рабочую систему не изменять без отдельной проверки.
