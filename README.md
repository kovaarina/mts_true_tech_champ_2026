# Go2 Robotics Challenge — Уровень 1 «Квалификация»

Отдельный пакет участника для **уровня 1**. Ты программируешь четвероногого робота **Unitree Go2**
в симуляторе **Webots R2025a**: робот реально ходит под физикой (штатная нейросеть-политика), ты
пишешь только высокоуровневое управление. Полное описание уровня — **[TASK.md](TASK.md)**,
справочник API робота и манипулятора — **[API.md](API.md)**.

## Твоё решение — папка controllers/participant/
```
controllers/participant/          # вся папка: модули, YAML-конфиги, participant.py (точка входа)
```
Всё остальное (мир, робот, локомоция `go2_api`, судья) фиксировано и одинаково для всех.

## Быстрый старт
```bash
# 1. окружение (создаёт .venv, ставит numpy+MNN, проверяет локомоцию)
./setup.sh                 # macOS / Linux
#   Windows (PowerShell): powershell -ExecutionPolicy Bypass -File .\setup.ps1

# 2. установи Webots R2025a — см. INSTALL.md

# 3. запусти
./run.sh                   # macOS / Linux
#   Windows: .\run.ps1
```

Откроется Webots с уровнем. Правь `participant.py`, сохраняй — Webots перезагрузит контроллер.
Счёт и события судьи — в консоли; итог — в `controllers/referee/result.json`.

Установка под Windows/Linux/macOS — **[INSTALL.md](INSTALL.md)**.

## Структура
```
common/                 # фиксированный движок: робот, локомоция (go2_api), судья, объекты
worlds/level1.wbt      # пример мира; на судействе — сгенерированная трасса, решению недоступна
config/level1.json     # пример трассы; на судействе карта ДРУГАЯ, решению этот файл НЕДОСТУПЕН
controllers/participant # ТВОЁ РЕШЕНИЕ — вся папка (модули, конфиги Nav2/SLAM, participant.py)
controllers/referee     # локальный скорер (двери/счёт/result.json)
setup.* run.* INSTALL.md API.md TASK.md
```
