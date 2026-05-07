#-----------------------------------------------------------------------------
#   Omar Fernandez-Contreras
#   xacademy
#   launcher.py
#
#   Runs on boot via systemd.
#   Watches MS5837 pressure — when float is placed in water, launches PID.py.
#   All events written to ~/launcher.log
#-----------------------------------------------------------------------------

import sys
import os
sys.path.insert(0, "/home/programming-pathway/MATE-ROV/ms5837")

import time
import ms5837
import subprocess


# config
PRESSURE_THRESHOLD_PA = 150   # pa above air baseline = float is in water, raise if false triggers
STABLE_READS          = 1     # consecutive reads needed before launching (~2 seconds)
CHECK_INTERVAL_S      = 0.5   # how often to poll while waiting

PID_SCRIPT = "/home/programming-pathway/MATE-ROV/Final_Product/PID.py"
LOG_FILE   = os.path.expanduser("~/launcher.log")


# ------
# logging
# ------

def log(msg):
    line = f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {msg}"
    print(line)
    with open(LOG_FILE, "a") as f:
        f.write(line + "\n")


# ------
# sensor
# ------

sensor = ms5837.MS5837_30BA(bus=1)

# initialises sensor and sets fluid density for pool water
def init_sensor():
    sensor.setFluidDensity(1000)
    if not sensor.init():
        log("sensor init failed"); return False
    if not sensor.read():
        log("sensor first read failed"); return False
    return True

# returns raw pressure in pa
def get_pressure():
    if not sensor.read():
        raise RuntimeError("sensor read failed")
    return sensor.pressure(ms5837.UNITS_Pa)


# ------
# launch

# hands off to PID.py, launcher exits after it finishes
def launch():
    log(f"launching {PID_SCRIPT}")
    result = subprocess.run(["/home/programming-pathway/MATE-ROV/venv/bin/python3", PID_SCRIPT])
    if result.returncode == 0:
        log("PID.py finished — check ~/MATE-ROV/Final_Product for csv")
    else:
        log(f"PID.py exited with code {result.returncode}")


# ------
# main

def main():
    log("=" * 40)
    log("float launcher started")
    log("=" * 40)

    if not init_sensor():
        log("sensor unavailable — cannot launch autonomously, exiting")
        return

    # lock in air baseline before the float touches water
    time.sleep(1)
    baseline     = get_pressure()
    stable_count = 0
    log(f"air baseline: {baseline:.1f} pa — waiting for water placement...")

    while True:
        try:
            pressure = get_pressure()
            delta    = pressure - baseline

            log(f"pressure: {pressure:.1f} pa  delta: {delta:+.1f} pa  stable: {stable_count}/{STABLE_READS}")

            if delta >= PRESSURE_THRESHOLD_PA:
                stable_count += 1
            else:
                stable_count = 0

            if stable_count >= STABLE_READS:
                log("water confirmed — launching")
                launch()
                return

        except RuntimeError as e:
            log(f"sensor error: {e} — retrying")
            stable_count = 0

        time.sleep(CHECK_INTERVAL_S)


if __name__ == "__main__":
    main()
