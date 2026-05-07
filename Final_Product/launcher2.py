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
sys.path.insert(0, "/home/programming-pathway/MATE-ROV/ms5837");

import time
import ms5837
import subprocess # what dis

# Config
PID_SCRIPT = "/home/programming-pathway/MATE-ROV/Final_Product/PID.py"
LOG_FILE = os.path.expanduser("~/launcher.log")


# -------
# Logging
# -------

def log(msg):
    line


# -------
# Sensor 
# -------

sensor = ms5837.MODEL_02BA(bus = 1)

def init_sensor():
    sensor.setFluidDensity(1000) # Value Depends on Water
    if not sensor.init():
        log("sensor init failed"); return False
    if not sensor.read():
        log("se