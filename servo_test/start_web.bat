@echo off
rem Servo web console launcher (Windows)
rem First run: creates .venv and installs requirements.txt. Later runs start right away.
setlocal
cd /d "%~dp0"
chcp 65001 >nul

set "PY="
where py >nul 2>nul && set "PY=py -3"
if not defined PY (
  where python >nul 2>nul && set "PY=python"
)
if not defined PY (
  echo Python 3.9+ is not installed. Install it from https://www.python.org/downloads/
  echo ^(check "Add python.exe to PATH" during setup^)
  pause
  exit /b 1
)

if not exist ".venv\Scripts\python.exe" (
  echo [setup] Creating virtual environment .venv ...
  %PY% -m venv .venv
  if errorlevel 1 (
    echo Failed to create .venv
    pause
    exit /b 1
  )
)

fc /b requirements.txt .venv\requirements.installed >nul 2>nul
if errorlevel 1 (
  echo [setup] Installing packages from requirements.txt ...
  ".venv\Scripts\python.exe" -m pip install --disable-pip-version-check -q -r requirements.txt
  if errorlevel 1 (
    echo Package install failed - check the internet connection
    pause
    exit /b 1
  )
  copy /y requirements.txt .venv\requirements.installed >nul
)

".venv\Scripts\python.exe" servo_web.py %*
if errorlevel 1 pause
