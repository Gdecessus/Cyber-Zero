# CyberZero — Pi Kiosk Deployment

This folder contains everything needed to deploy CyberZero to a Raspberry Pi 5
as a stand-alone touchscreen kiosk. After deployment, the Pi boots straight
into the chess UI — no keyboard, no mouse, no terminal needed.

## What's in this folder

| File | Purpose |
|------|---------|
| `setup-pi.sh` | One-shot installer — run once on the Pi |
| `cyberzero.service` | systemd unit file — runs the Flask server on boot |
| `cyberzero-kiosk.desktop` | Auto-launches Chromium in kiosk mode on login |
| `README.md` | This file |

## Hardware required

- Raspberry Pi 5 (4GB or 8GB)
- 10.1" HDMI touchscreen (1920x1200 or similar)
- Micro-HDMI to HDMI cable (Pi 5 → screen)
- USB cable (touch input from screen → Pi)
- Power supply for the screen
- Pi 5 USB-C power supply
- microSD card (32GB+) flashed with **Raspberry Pi OS Bookworm 64-bit**

## One-time setup (with keyboard temporarily attached)

1. **Flash microSD** with Raspberry Pi OS Bookworm 64-bit (use the Imager).
   In the Imager's advanced options:
   - Set hostname: `cyberzero`
   - Set username: `pi`, password: anything you'll remember
   - Enable SSH (so you can recover remotely if anything breaks)
   - Set wifi credentials

2. **Boot the Pi** with keyboard, mouse, screen attached. Open a terminal.

3. **Get the project onto the Pi.** Either:
   ```bash
   # Option A: from GitHub
   cd ~
   git clone https://github.com/Gdecessus/Cyber-Zero.git
   ```
   ```bash
   # Option B: from a USB stick
   cp -r /media/pi/USB/Cyber-Zero ~/
   ```

4. **Run the setup script:**
   ```bash
   cd ~/Cyber-Zero
   chmod +x deploy/setup-pi.sh
   bash deploy/setup-pi.sh
   ```
   The script installs Python dependencies, the systemd service, the Chromium
   kiosk autostart, disables screen blanking, and enables auto-login. Takes
   about 5–10 minutes (TensorFlow install is the slow bit).

5. **Reboot:**
   ```bash
   sudo reboot
   ```

6. **Verify** — the Pi should boot, auto-login, and ~30 seconds later show the
   chess UI fullscreen on the touchscreen. Tap a piece and tap a destination
   to play.

7. **You can now disconnect the keyboard.** From this point onwards the Pi
   runs as a pure touchscreen appliance.

## Demo usage

1. Plug the Pi into power.
2. Wait ~30 seconds for boot + service + Chromium.
3. Touch the board to play.
4. To shut down cleanly between demos: SSH in from your phone and run
   `sudo shutdown now`, or just unplug power.

## Troubleshooting

### The game UI doesn't appear after boot

SSH from your phone or laptop on the same wifi:
```bash
ssh pi@cyberzero.local
# or
ssh pi@<pi-ip>
```

Check the Flask server:
```bash
sudo systemctl status cyberzero
```

Check the live logs:
```bash
sudo journalctl -u cyberzero -f
```

If the server is running but Chromium didn't launch:
```bash
DISPLAY=:0 chromium-browser --kiosk http://localhost:8000 &
```

### The AI is too slow on the Pi

Pi 5 has no GPU, so TensorFlow runs on CPU. The current `n_sims=200` will take
20–40 seconds per AI move. If you need faster moves for the demo, edit
`src/server/server.py` and reduce:
```python
self.mcts = MCTS(self.model, n_sims=50)  # was 200
```
Then restart: `sudo systemctl restart cyberzero`.

### Screen goes blank or shows mouse cursor

`setup-pi.sh` handles this, but if the cursor reappears or the screen sleeps,
re-run the script.

### How to exit kiosk mode (need keyboard)

`Alt+F4` closes Chromium and drops you back to the desktop.
`Ctrl+Alt+T` opens a terminal.

## File structure on the Pi after setup

```
/home/pi/Cyber-Zero/                       ← project root
├── src/                                   ← Python source
├── UI_chess/                              ← static web assets
├── deploy/                                ← (this folder)
│   ├── setup-pi.sh
│   ├── cyberzero.service
│   ├── cyberzero-kiosk.desktop
│   └── README.md
└── requirements.txt

/etc/systemd/system/cyberzero.service     ← copied here by setup-pi.sh
/home/pi/.config/autostart/
└── cyberzero-kiosk.desktop               ← copied here by setup-pi.sh
```

## Boot sequence

```
0s    Pi powers on, hardware init
~10s  Pi auto-logs into the LXDE desktop
~12s  systemd starts cyberzero.service in the background
        - Loads TensorFlow, builds ChessModel (random weights or best_model.keras if present)
        - Starts Flask on 0.0.0.0:8000
~20s  Chromium kiosk autostart fires (after its 8s sleep)
        - Connects to http://localhost:8000
~30s  Game UI fills the touchscreen — ready to play
```
