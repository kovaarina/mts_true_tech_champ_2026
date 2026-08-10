﻿# Установка окружения (Windows, PowerShell). Один раз перед запуском.
#   powershell -ExecutionPolicy Bypass -File .\setup.ps1
# Создаёт локальный .venv, ставит numpy+MNN, прогоняет проверку локомоции (без Webots).
# Webots ставится отдельно — см. INSTALL.md.
$ErrorActionPreference = "Stop"
$HERE = Split-Path -Parent $MyInvocation.MyCommand.Path

# ищем интерпретатор Python 3.10+
$PY = $null
foreach ($cand in @("py -3", "python", "python3")) {
  $exe, $arg = $cand.Split(" ", 2)
  if (Get-Command $exe -ErrorAction SilentlyContinue) { $PY = $cand; break }
}
if (-not $PY) { Write-Error "Не найден Python. Установи Python 3.10+ (см. INSTALL.md)."; exit 1 }

$ok = & ([scriptblock]::Create("$PY -c `"import sys; sys.exit(0 if sys.version_info>=(3,10) else 1)`""))
if ($LASTEXITCODE -ne 0) { Write-Error "Нужен Python 3.10+."; exit 1 }

Write-Host ">> создаю виртуальное окружение .venv"
& ([scriptblock]::Create("$PY -m venv `"$HERE\.venv`""))
$VPY = Join-Path $HERE ".venv\Scripts\python.exe"

Write-Host ">> обновляю pip и ставлю зависимости"
& $VPY -m pip install --upgrade pip | Out-Null
& $VPY -m pip install -r (Join-Path $HERE "requirements.txt")
if ($LASTEXITCODE -ne 0) { Write-Error "pip install не удался."; exit 1 }

Write-Host ">> проверка локомоции (numpy + MNN + модель ходьбы, без Webots)"
& $VPY (Join-Path $HERE "common\locomotion\policy_walk.py")
if ($LASTEXITCODE -ne 0) { Write-Error "Проверка локомоции не прошла."; exit 1 }

Write-Host ""
Write-Host "OK. Дальше:"
Write-Host "  1) установи Webots R2025a (см. INSTALL.md)"
Write-Host "  2) запусти уровень:  .\run.ps1 1   (или 2 / 3)"
