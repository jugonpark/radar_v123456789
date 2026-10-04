#!/usr/bin/env bash
set -euo pipefail
echo "Installing Raspberry Pi radar prerequisites..."
sudo apt update
sudo apt install -y python3 python3-pip python3-venv python3-tk git usbutils
if ! id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
  echo "Adding $USER to dialout; log out/in before using the radar."
  sudo usermod -aG dialout "$USER"
fi
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements-pi.txt
echo "Setup complete. Tkinter comes from apt, not pip."
