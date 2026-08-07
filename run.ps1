# Запуск уровня в Webots (Windows). Требует setup.ps1 и установленный Webots R2025a (INSTALL.md).
#   powershell -ExecutionPolicy Bypass -File .\run.ps1
$ErrorActionPreference = "Stop"
$HERE = Split-Path -Parent $MyInvocation.MyCommand.Path
$VSCRIPTS = Join-Path $HERE ".venv\Scripts"
if (-not (Test-Path (Join-Path $VSCRIPTS "python.exe"))) { Write-Error "нет .venv — сначала setup.ps1"; exit 1 }
$env:PATH = "$VSCRIPTS;" + $env:PATH
$site = Join-Path $HERE ".venv\Lib\site-packages"
if (Test-Path $site) { $env:PYTHONPATH = "$site;" + $env:PYTHONPATH }
$webots = $env:WEBOTS
if (-not $webots) {
  $cands = @(
    (Join-Path $env:ProgramFiles "Webots\msys64\mingw64\bin\webotsw.exe"),
    (Join-Path $env:ProgramFiles "Webots\msys64\mingw64\bin\webots.exe"),
    (Join-Path ${env:ProgramFiles(x86)} "Webots\msys64\mingw64\bin\webotsw.exe")
  )
  if ($env:WEBOTS_HOME) { $cands = @((Join-Path $env:WEBOTS_HOME "msys64\mingw64\bin\webotsw.exe")) + $cands }
  foreach ($c in $cands) { if ($c -and (Test-Path $c)) { $webots = $c; break } }
  if (-not $webots) { $g = Get-Command webots -ErrorAction SilentlyContinue; if ($g) { $webots = $g.Source } }
}
if (-not $webots -or -not (Test-Path $webots)) {
  Write-Error "Webots не найден. Установи R2025a (INSTALL.md) или задай `$env:WEBOTS = 'C:\путь\webotsw.exe'"
  exit 1
}
& $webots (Join-Path $HERE "worlds\level1.wbt")
