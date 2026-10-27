#!/bin/sh
# Servo web console launcher (macOS / Linux)
# First run: creates .venv and installs requirements.txt. Later runs start right away.
# Usage: sh start_web.sh [servo_web.py options]
set -e
cd "$(dirname "$0")"

PY=$(command -v python3 || command -v python || true)
if [ -z "$PY" ]; then
  echo "Python 3.9+ is not installed (macOS: brew install python, Ubuntu: sudo apt install python3 python3-venv)"
  exit 1
fi

if [ ! -x .venv/bin/python ]; then
  echo "[setup] Creating virtual environment .venv ..."
  "$PY" -m venv .venv || { echo "Failed to create .venv (Ubuntu/Debian: sudo apt install python3-venv)"; exit 1; }
fi

if ! cmp -s requirements.txt .venv/requirements.installed; then
  echo "[setup] Installing packages from requirements.txt ..."
  .venv/bin/python -m pip install --disable-pip-version-check -q -r requirements.txt
  cp requirements.txt .venv/requirements.installed
fi

if [ "$(uname)" = "Linux" ] && ! id -nG | grep -qwE 'dialout|uucp'; then
  echo "Note: to open the serial port without sudo, run: sudo usermod -aG dialout \$USER  (then log out and back in)"
fi

exec .venv/bin/python servo_web.py "$@"
