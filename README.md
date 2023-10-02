# VoltBro Cores

> **`git clone --recurse-submodules -j8 git@github.com:voltbro/vbcores.git`**
>
>  В этом репозитории есть сабмодули!

*Базовое окружение со всеми нужными инструментами для работы с платами VoltBro.*

## Настройка

Полная установка: `./configure.sh` - все пакеты apt + python, добавляет переменные в .bashrc, node + zx (можно не ставить node - `NO_NODE_INSTALL=Y ./configure.sh`).

<details>
  <summary>Ручная установка</summary>

  1. Добавьте в `.bashrc`:
  ```bash
  export PATH="$PATH:/home/pi/.local/bin"

  # Этот if -  опционален, но очень удобен
  if [[ -n $BASH_INIT_COMMAND ]]; then
      echo "Running: $BASH_INIT_COMMAND"
      eval "$BASH_INIT_COMMAND"
  fi

  export YAKUT_FORMAT=json
  export UAVCAN__CAN__IFACE=socketcan:can0
  export UAVCAN__NODE__ID=
  export UAVCAN__CAN__MTU=64
  export UAVCAN__CAN__BITRATE="1000000 800000"
  ```

  2. Пакеты и утилиты
  ```bash
  python3 -m pip install --user pipx && python3 -m pip ensurepath
  sudo apt install libsdl2-dev libasound2-dev libjack-dev can-utils jq jsonnet
  pipx install yakut[joystick]
  pipx install nunavut
  ```
  <details>
    <summary>Зависимости для скриптов</summary>

    Node + ZX
    ```
    curl -fsSL https://deb.nodesource.com/setup_19.x | sudo -E bash - &&\
    sudo apt-get install -y nodejs
    sudo npm i -g zx
    ```

  </details>

  3. `python3 -m pip install -r requirements.dev.txt`. Перед этим шагом можно создать виртуальное окружение, но если вы работаете, например, на raspberry pi, то это не всегда имеет смысл, поэтому оставляем это на ваше усмотрение.
</details>

Проверка что все хорошо: `make run-script` -> `dev` -> `compile.py` (занимает некоторое время).

Для удобства, можно через vscode `Файл->открыть рабочую область из файла` открыть `папка_репозитория/.vscode/vbcores.code-workspace`, там есть полезные настройки. Без этого - в терминале из которого вы работаете с проектом первой командой выполняйте `source setup.sh`, или как-то иначе настройте выполнение этих команд автоматически.

## Описание

**Примеры проектов для плат VoltBro в папке [examples](./examples/).**

### Структура репозитория

- [external](./external/) - исходники сторонних библиотек
- [scripts](./scripts/) - скрипты (настройка `can`, компиляция `dsdl` и т.д.)
- [common](./common/) - общие для проектов ресурсы (билды `dsdl`, подключаемые библиотеки и т.д.). Рассчет на то, что эту папку можно напрямую линковать (в смысле `ln`, а не `ld`) в ваш проект и *it just works*.
- [voltbro_can_py](./voltbro_can_py/) - питоновская инфраструктура для работы с `cyphal` и пр.

### Makefile

Содержит часто используемые команды (`make help` - выведет список).

- `run-script` - утилита для удобного запуска скриптов
- `compile-types` - компиляция `dsdl`
- `lint` - линтинг python кода (TODO)
- `format` - автоформаттер python кода (TODO)

### CANFD-HAT

https://wiki.seeedstudio.com/2-Channel-CAN-BUS-FD-Shield-for-Raspberry-Pi/#software

1. `sudo nano /boot/config.txt` -> добавить `dtoverlay=seeed-can-fd-hat-v2`
2. `sudo reboot`
3. `ifconfig -a` - должен показывать интерфейсы `can0`, `can1`
4.  Если все ок - `make run-script` -> `dev` -> `canhat.mjs` (если node+zx нет: `sudo ip link set can0 up type can bitrate 1000000 dbitrate 8000000 restart-ms 1000 berr-reporting on fd on`, аналогично для `can1`)

Опционально: чтоб тестировать получение сообщений, можно подключить каналы на шилде друг к другу - `0_L <===> 1_L`, `0_H <===> 1_H`, но это иногда приводит к странным побочным эффектам, наподобие того что все сообщения видно на любом трансивере в сети.

После каждой перезагрузки настройку `can` интерфейса придется выполнять заново. Чтоб плата настраивалась автоматически, установите юнит, выполняющийся при старте платы:

1. `curl https://raw.githubusercontent.com/voltbro/vbcores-units/master/download_and_install.sh | bash` - качает и устанавливает все необходимые компоненты
2. `sudo systemctl enable board && sudo systemctl start board` - запускает сервис

### Работа с CYPHAL

*См. [docs/CYPHAL.md](./docs/CYPHAL.md)*
