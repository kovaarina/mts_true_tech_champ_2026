#!/usr/bin/env bash
# Установка окружения (macOS / Linux). Один раз перед запуском.
#   ./setup.sh
# Создаёт локальный .venv, ставит numpy+MNN, прогоняет проверку локомоции (без Webots).
# Webots ставится отдельно — см. INSTALL.md.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"

PY="${PYTHON:-python3}"
if ! command -v "$PY" >/dev/null 2>&1; then
  echo "ОШИБКА: не найден python3. Установи Python 3.10+ (см. INSTALL.md)." >&2
  exit 1
fi
"$PY" - <<'PYCHK' || { echo "ОШИБКА: нужен Python 3.10+." >&2; exit 1; }
import sys
raise SystemExit(0 if sys.version_info >= (3, 10) else 1)
PYCHK

echo ">> создаю виртуальное окружение .venv"
"$PY" -m venv "$HERE/.venv"
VPY="$HERE/.venv/bin/python"

echo ">> обновляю pip и ставлю зависимости"
"$VPY" -m pip install --upgrade pip >/dev/null
"$VPY" -m pip install -r "$HERE/requirements.txt"

echo ">> проверка локомоции (numpy + MNN + модель ходьбы, без Webots)"
"$VPY" "$HERE/common/locomotion/policy_walk.py"

echo
echo "OK. Дальше:"
echo "  1) установи Webots R2025a (см. INSTALL.md)"
echo "  2) запусти уровень:  ./run.sh 1   (или 2 / 3)"
