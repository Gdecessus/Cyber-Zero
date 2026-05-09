#!/bin/bash
# ============================================================
# CyberZero Pi Kiosk Setup
# Run this ONCE on a fresh Raspberry Pi 5 (Bookworm 64-bit).
# After this, the Pi boots straight into the chess game UI
# in Chromium kiosk mode. No keyboard needed afterwards.
# ============================================================

set -e   # exit immediately on error

PROJECT_DIR="/home/pi/Cyber-Zero"

echo "============================================"
echo "  CyberZero Pi Setup"
echo "============================================"

# ---- 1. Verify project is in the right place ----
if [ ! -d "$PROJECT_DIR" ]; then
    echo "ERROR: Project not found at $PROJECT_DIR"
    echo "Copy or git clone the project to that path first."
    exit 1
fi

# ---- 2. Install system packages ----
echo ""
echo "[1/6] Installing system packages..."
sudo apt update
sudo apt install -y python3-pip chromium-browser unclutter

# ---- 3. Install Python dependencies ----
echo ""
echo "[2/6] Installing Python dependencies..."
cd "$PROJECT_DIR"
pip3 install --break-system-packages -r requirements.txt

# ---- 4. Install systemd service ----
echo ""
echo "[3/6] Installing systemd service..."
sudo cp deploy/cyberzero.service /etc/systemd/system/cyberzero.service
sudo systemctl daemon-reload
sudo systemctl enable cyberzero
sudo systemctl start cyberzero

# ---- 5. Install Chromium kiosk autostart ----
echo ""
echo "[4/6] Installing Chromium kiosk autostart..."
mkdir -p /home/pi/.config/autostart
cp deploy/cyberzero-kiosk.desktop /home/pi/.config/autostart/cyberzero-kiosk.desktop

# ---- 6. Disable screen blanking and enable auto-login ----
echo ""
echo "[5/6] Configuring desktop auto-login and disabling screen blanking..."
sudo raspi-config nonint do_boot_behaviour B4   # Desktop auto-login as 'pi'
sudo raspi-config nonint do_blanking 1          # Disable screen blanking

# ---- 7. Hide mouse cursor ----
echo ""
echo "[6/6] Hiding mouse cursor when idle..."
mkdir -p /home/pi/.config/lxsession/LXDE-pi
AUTOSTART="/home/pi/.config/lxsession/LXDE-pi/autostart"
if [ ! -f "$AUTOSTART" ]; then
    touch "$AUTOSTART"
fi
if ! grep -q "unclutter" "$AUTOSTART"; then
    echo "@unclutter -idle 0" >> "$AUTOSTART"
fi

# ---- Done ----
echo ""
echo "============================================"
echo "  SETUP COMPLETE"
echo "============================================"
echo ""
echo "Verify the server is running:"
echo "  systemctl status cyberzero"
echo ""
echo "Reboot to test the kiosk:"
echo "  sudo reboot"
echo ""
echo "After reboot, the Pi will:"
echo "  1. Auto-login to desktop"
echo "  2. Start the Flask server in the background"
echo "  3. Launch Chromium fullscreen at localhost:8000"
echo "  4. Hide the mouse cursor"
echo ""
echo "If anything goes wrong, SSH in from another device:"
echo "  ssh pi@$(hostname -I | awk '{print $1}')"
echo ""
