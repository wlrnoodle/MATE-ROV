# Vertical Profiling Float
**Omar Fernandez-Contreras — xacademy**

Autonomous buoyancy-controlled float for vertical ocean/pool profiling.
Descends to 2.5 m, holds 30 s, ascends to 40 cm, holds 30 s — twice.
All depth data is saved to CSV on the Pi and graphed after recovery.

---

## Hardware

| Component | Detail |
|---|---|
| Computer | Raspberry Pi (Ubuntu) |
| Depth sensor | MS5837-30BA |
| Actuator | DC syringe motor via PWM |
| PWM pin | GPIO 13 |
| Launch button | GPIO 16 → GND |

### Sensor wiring (I2C bus 1)
```
Pi pin 1  (3.3V) → sensor VCC
Pi pin 3  (SDA)  → sensor SDA
Pi pin 5  (SCL)  → sensor SCL
Pi pin 6  (GND)  → sensor GND
```


### Syringe spec
| | |
|---|---|
| Total volume | 200 ml |
| Volume to float | 157 ml |
| Sink stroke | 43 ml |
| PWM retract (rise) | 1050 µs |
| PWM extend (sink) | 1950 µs |
| PWM stop | 1500 µs |

---

## File structure

```
~/float_project/
    PID.py          ← main dive controller
    launcher.py     ← runs on boot, detects water, launches PID.py
    README.md       ← this file

~/ms5837/           ← MS5837 library (clone from GitHub)
~/float_data/       ← CSV data files created automatically after each run
```

---

## First-time setup

### 1. Clone the MS5837 library
```bash
cd ~
git clone https://github.com/bluerobotics/ms5837-python ms5837
```

### 2. Install dependencies
```bash
sudo apt update
sudo apt install -y python3-pip python3-pigpio pigpio
pip3 install RPi.GPIO matplotlib
```

### 3. Enable I2C on the Pi
```bash
sudo raspi-config
# Interface Options → I2C → Enable
```

Verify the sensor is visible on the bus:
```bash
i2cdetect -y 1
# You should see address 0x76 or 0x77 appear in the grid
```

### 4. Start pigpio daemon (required for PWM)
```bash
sudo pigpiod
```

To start it automatically on every boot:
```bash
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

### 5. Copy project files to Pi
```bash
# From your computer
scp -r float_project/ pi@<pi-ip>:~/float_project/
```

Or clone/copy directly on the Pi if you have the files there already.

---

## Running manually (testing)

```bash
# Always start pigpio first
sudo pigpiod

# Run the main dive script directly
python3 ~/float_project/PID.py
```

The script will:
1. Lock in the surface pressure
2. Retract syringe to float position
3. Run profile 1 (descend → hold → ascend → hold)
4. Surface pause 5 s
5. Run profile 2
6. Save CSV to `~/float_data/`
7. Display final depth-over-time graph

---

## Autonomous boot setup (competition mode)

Set up the systemd service so the launcher runs automatically on power-on.

### 1. Create the service file
```bash
sudo nano /etc/systemd/system/float.service
```

Paste:
```ini
[Unit]
Description=Autonomous float launcher
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi
ExecStartPre=/bin/sleep 10
ExecStartPre=/usr/bin/pigpiod
ExecStart=/usr/bin/python3 /home/pi/float_project/launcher.py
Restart=on-failure
RestartSec=5
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

> **Note:** if your Ubuntu username is not `pi`, replace all instances of `pi`
> with your actual username. Check with `whoami`.

### 2. Enable and start the service
```bash
sudo systemctl daemon-reload
sudo systemctl enable float.service
sudo systemctl start float.service
```

### 3. Verify it is running
```bash
sudo systemctl status float.service
```

### 4. Watch live logs
```bash
# systemd journal
journalctl -u float.service -f

# or the launcher log file
tail -f ~/launcher.log
```

---

## Competition day checklist

- [ ] Pi fully charged / powered
- [ ] `sudo systemctl status float.service` shows **active**
- [ ] Sensor visible on I2C: `i2cdetect -y 1`
- [ ] Syringe moves when tested manually
- [ ] `~/float_data/` folder exists (created automatically on first run)
- [ ] Power on Pi → wait ~15 s for full boot
- [ ] Place float in water **or** press GPIO 16 button
- [ ] Float dives automatically — do not touch
- [ ] Recover float after both profiles complete
- [ ] Pull data off Pi (see below)

---

## How autonomous launch works

`launcher.py` runs on boot and does two things at once:

**Primary — water detection:**
Reads the MS5837 pressure continuously. When it sees a pressure rise of
≥ 150 Pa above the air baseline for 4 consecutive reads (~2 seconds), it
confirms the float is in water and launches `PID.py`.


To adjust sensitivity edit these values at the top of `launcher.py`:
```python
PRESSURE_THRESHOLD_PA = 150   # raise if getting false triggers, lower if not detecting
STABLE_READS          = 4     # number of consecutive reads needed before launching
```

---

## Retrieving data after recovery

All depth readings are saved to CSV in `~/float_data/` on the Pi.
Each run creates a new timestamped file: `run_YYYYMMDD_HHMMSS.csv`

### Option 1 — copy over WiFi (Pi must be on same network)
```bash
scp pi@<pi-ip>:~/float_data/*.csv ~/Desktop/
```


### Option 3 — read directly on Pi
```bash
cat ~/float_data/run_<timestamp>.csv
```

### CSV format
```
timestamp_s, elapsed_s, depth_m, profile
1713900000.12, 0.00, 0.012, 1
1713900000.22, 0.10, 0.045, 1
...
```

| Column | Description |
|---|---|
| `timestamp_s` | Unix timestamp (seconds since epoch) |
| `elapsed_s` | Seconds since dive started |
| `depth_m` | Depth in metres (positive = submerged) |
| `profile` | 1 = first dive, 2 = second dive |

---

## PID tuning

The PID gains are set in `PID.py`:
```python
kp, ki, kd = 10.0, 0.5, 2.0
```

These will need tuning based on  robot's actual mass and syringe response.
Use this process:

1. Set `ki=0, kd=0` — tune `kp` until robot reaches depth without wild oscillation
2. Add `kd` (start ~1.0) to reduce overshoot on approach
3. Add small `ki` (start ~0.1) to fix any steady-state depth error

Watch the printed output during a test dive — `err` should shrink and stabilise,
`ctrl` should settle near 0 when holding depth.

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `pigpio daemon not running` | `sudo pigpiod` then retry |
| Sensor not found on I2C | Check wiring, run `i2cdetect -y 1`, confirm address 0x76/0x77 |
| Float goes wrong direction | Swap `PW_RETRACT` and `PW_EXTEND` values in `PID.py` |
| Water detection too sensitive | Raise `PRESSURE_THRESHOLD_PA` in `launcher.py` |
| Water not detected | Lower `PRESSURE_THRESHOLD_PA` or just use the button |
| Service not starting | Check username in float.service matches `whoami` output |
| No CSV files created | Check `~/float_data/` exists and Pi has write permission |
| Live graph not showing | Run with a monitor attached or disable live plot for headless use |
