#!/usr/bin/env bash
# Запуск уровня в Webots (macOS / Linux). Требует ./setup.sh и установленный Webots R2025a (INSTALL.md).
#   ./run.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
VBIN="$HERE/.venv/bin"
[ -x "$VBIN/python3" ] || { echo "нет .venv — сначала ./setup.sh" >&2; exit 1; }
# Webots зовёт контроллер как python3 из PATH — ставим venv первым, чтобы это был наш python с numpy+MNN.
export PATH="$VBIN:$PATH"
SITE="$(ls -d "$HERE"/.venv/lib/python3.*/site-packages 2>/dev/null | head -1 || true)"
[ -n "${SITE:-}" ] && export PYTHONPATH="${SITE}${PYTHONPATH:+:$PYTHONPATH}"
WEBOTS="${WEBOTS:-}"
if [ -z "$WEBOTS" ]; then
  for c in /Applications/Webots.app/Contents/MacOS/webots /usr/local/bin/webots /usr/bin/webots /snap/bin/webots "$(command -v webots 2>/dev/null || true)"; do
    [ -n "$c" ] && [ -x "$c" ] && WEBOTS="$c" && break
  done
fi
[ -n "$WEBOTS" ] && [ -x "$WEBOTS" ] || { echo "Webots не найден. Установи R2025a (INSTALL.md) или задай WEBOTS=/путь/к/webots ./run.sh" >&2; exit 1; }
exec "$WEBOTS" "$HERE/worlds/level1.wbt"
