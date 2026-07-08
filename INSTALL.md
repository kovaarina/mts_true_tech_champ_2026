# Установка (Windows · Linux · macOS)

Нужны две вещи: **Python 3.10–3.13** и **Webots R2025a**. Дальше — `setup` (создаёт `.venv` и
ставит зависимости) и `run` (запускает уровень). Ниже — по шагам под каждую ОС.

Официальные загрузки Webots R2025a (одна и та же версия для всех — важно для совместимости):
<https://github.com/cyberbotics/webots/releases/tag/R2025a>

---

## Windows

1. **Python.** Скачай установщик с <https://www.python.org/downloads/windows/> (3.10–3.13).
   При установке отметь **«Add python.exe to PATH»**. Проверь в PowerShell: `py -3 --version`.
2. **Webots.** Скачай `webots-R2025a_setup.exe` со страницы релиза (ссылка выше) и установи
   (по умолчанию в `C:\Program Files\Webots`).
3. **Окружение.** В PowerShell из папки репозитория:
   ```powershell
   powershell -ExecutionPolicy Bypass -File .\setup.ps1
   ```
4. **Запуск:**
   ```powershell
   .\run.ps1 1        # уровень 1  (или 2 / 3)
   ```
   Если Webots установлен не в стандартную папку — задай путь:
   `$env:WEBOTS = "C:\путь\к\Webots\msys64\mingw64\bin\webotsw.exe"; .\run.ps1 1`

---

## Linux (Ubuntu / Debian)

1. **Python + venv:**
   ```bash
   sudo apt update && sudo apt install -y python3 python3-venv python3-pip
   ```
2. **Webots.** Скачай `webots_2025a_amd64.deb` со страницы релиза и поставь:
   ```bash
   sudo apt install -y ./webots_2025a_amd64.deb
   ```
   (Альтернатива: `sudo snap install webots`.)
3. **Окружение и запуск:**
   ```bash
   ./setup.sh
   ./run.sh 1         # уровень 1  (или 2 / 3)
   ```
   Если `webots` не в PATH: `WEBOTS=/path/to/webots ./run.sh 1`.

> Без дисплея (сервер/CI) запускай под виртуальным экраном:
> `xvfb-run -a ./run.sh 1`.

---

## macOS

1. **Python.** Через [Homebrew](https://brew.sh): `brew install python@3.12`
   (или установщик с <https://www.python.org/downloads/macos/>). Проверь: `python3 --version`.
2. **Webots.** Скачай `webots-R2025a.dmg` со страницы релиза, открой и перетащи **Webots.app**
   в `/Applications`. Первый запуск: правый клик → *Open* (обойти Gatekeeper).
3. **Окружение и запуск:**
   ```bash
   ./setup.sh
   ./run.sh 1         # уровень 1  (или 2 / 3)
   ```
   Работает и на Apple Silicon (M1–M4), и на Intel — колёса MNN есть под обе архитектуры.

---

## Как это устроено (и типичные проблемы)

- `setup` создаёт локальный **`.venv`** и ставит в него `numpy` и `MNN` — на них крутится
  политика ходьбы робота. В конце прогоняется проверка `common/locomotion/policy_walk.py`
  (должна вывести `... — OK`), она **не требует Webots**.
- `run` ставит `.venv` первым в `PATH`, поэтому команда `python3`, которой Webots запускает твой
  контроллер, разрешается в venv-питон с `numpy`/`MNN` — нативно, без конфликтов по ABI (Webots
  R2025a игнорирует `runtime.ini`, так что механизм именно через `PATH`).
- **`ModuleNotFoundError: numpy/MNN`** → не выполнен `setup`, либо запускаешь Webots не через
  `run.sh`/`run.ps1`: проверь, что `.venv` создан, и запускай уровень именно скриптом `run`.
- **`Webots not found`** → задай путь переменной `WEBOTS` (см. запуск выше) или доустанови Webots.
- **Робот стоит на месте** → это стартовый `participant.py`-каркас: он просто едет вперёд.
  Логику прохождения пишешь ты (см. `levelN/TASK.md` и `API.md`).
- Версии Python 3.10–3.13 — под них есть колёса MNN. На 3.14+ колёс пока может не быть.
