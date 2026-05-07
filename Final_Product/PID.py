#-----------------------------------------------------------------------------
#
#   Omar Fernandez-Contreras
#   xacademy
#   PID.py
#
#   Vertical profiling float — competition build
#
#   Hardware:
#     Sensor  : MS5837-30BA on I2C bus 1
#                 Pin 1  → 3.3V
#                 Pin 3  → SDA
#                 Pin 5  → SCL
#                 Pin 6  → GND
#     Actuator: DC syringe motor via PWM on GPIO pin 13
#                 Pulse range 1050 µs (retract/rise) – 1950 µs (extend/sink)
#
#   Syringe spec:
#     Total volume  : 200 ml
#     Float volume  : 157 ml  (minimum to stay buoyant)
#     Sink stroke   :  43 ml  (extra volume to sink)
#
#   Mission profile (runs twice — two vertical profiles):
#     1. Descend to 2.5 m, hold for 30 s
#     2. Ascend to 40 cm,  hold for 30 s
#     Safety rule: NEVER go shallower than 40 cm (–5 point penalty)
#
#   PID tuning notes:
#     kp — proportional gain, main response strength
#     ki — integral gain, removes steady-state error over time
#     kd — derivative gain, dampens overshoot
#     Start with ki=0, kd=0 and tune kp first.
#     These values will need adjustment based on actual robot behaviour.
#
#-----------------------------------------------------------------------------

import sys
import os
sys.path.insert(0, "/home/programming-pathway/MATE-ROV/ms5837")

import time
import ms5837
import lgpio
import matplotlib.pyplot as plt
from collections import deque


#──── ACTUATOR — lgpio PWM syringe on GPIO 13 ──────────────────────────────────────────────────────────────

GPIO_PIN  = 13
PWM_FREQ  = 50       # Hz — standard servo frequency

PULSE_MIN = 1950     # µs — fully retracted (157 ml in) → float rises
PULSE_MAX = 1050    # µs — fully extended  (200 ml in) → float sinks

SYRINGE_MAX_ML  = 200.0
FLOAT_NEEDED_ML = 157.0
SINK_STROKE_ML  = SYRINGE_MAX_ML - FLOAT_NEEDED_ML   # 43 ml

# Open lgpio chip handle at startup
try:
    _h = lgpio.gpiochip_open(0)
    lgpio.gpio_claim_output(_h, GPIO_PIN)
except Exception as e:
    sys.exit(f"ERROR: Could not open GPIO: {e}\nTry running with sudo.")


def set_position(pulse_us: int) -> None:
    """
    Move actuator to a position specified by pulse width in microseconds.
    Clamps pulse_us to PULSE_MIN–PULSE_MAX range.

    Args:
        pulse_us (int): target pulse width in µs
    """
    pulse_us = max(PULSE_MIN, min(PULSE_MAX, pulse_us))
    lgpio.tx_servo(_h, GPIO_PIN, pulse_us, PWM_FREQ)


def position_to_pulse(position_mm: float) -> int:
    """
    Convert a position in mm (0–100) to a PWM pulse width in µs.
    0 mm   = fully retracted = PULSE_MIN (float rises)
    100 mm = fully extended  = PULSE_MAX (float sinks)

    Args:
        position_mm (float): target position 0–100 mm

    Returns:
        int: pulse width in µs
    """
    position_mm = max(0.0, min(100.0, position_mm))
    fraction = position_mm / 100.0
    return int(PULSE_MIN + fraction * (PULSE_MAX - PULSE_MIN))


def move_to_mm(position_mm: float) -> None:
    """
    Move actuator to an absolute position in millimetres (0–100).
    0 mm   = fully retracted → float rises
    100 mm = fully extended  → float sinks

    Args:
        position_mm (float): target position in mm
    """
    pulse = position_to_pulse(position_mm)
    ml_displaced = FLOAT_NEEDED_ML + (position_mm / 100.0 * SINK_STROKE_ML)
    print(f"  Moving to {position_mm:.1f} mm  ({pulse} µs  "
          f"{ml_displaced:.1f} ml displaced)")
    set_position(pulse)


def set_syringe(percent: float) -> None:
    """
    Drive the syringe from PID controller output.

    Maps –100…+100% directly to 0…100 mm actuator travel.
    Positive percent → extend → more water displaced → float sinks.
    Negative percent → retract → less water displaced → float rises.
    """
    percent = max(-100.0, min(100.0, percent))
    # Remap –100…+100 → 0…100 mm
    position_mm = (percent + 100.0) / 2.0
    move_to_mm(position_mm)


def syringe_stop() -> None:
    """Hold actuator at current position (midpoint pulse — motor stops)."""
    set_position(int((PULSE_MIN + PULSE_MAX) / 2))
    print("  Syringe stopped (holding position).")


def syringe_off() -> None:
    """
    Turn off PWM signal entirely.
    Actuator holds last physical position, no holding torque.
    """
    lgpio.tx_servo(_h, GPIO_PIN, 0)
    print("  PWM signal off — actuator free.")


def gpio_cleanup() -> None:
    """Release GPIO handle cleanly on exit."""
    syringe_off()
    lgpio.gpiochip_close(_h)


sensor = ms5837.MS5837_30BA(bus=1)

surface_pressure = None   # Pa — captured at power-on, before submerging

#   Read current depth in metres below the power-on surface.

def depth_m():
    for attempt in range(3):
        if sensor.read():
            return (sensor.pressure(ms5837.UNITS_Pa) - surface_pressure) / (1000.0 * 9.80665)
        time.sleep(0.05)
    raise RuntimeError("sensor read failed after 3 attempts")


#──── PID CONTROLLER ──────────────────────────────────────────────────────────────────

def pid_controller(setpoint, pv, kp, ki, kd, previous_error, integral, dt):
    """
    Standard discrete PID controller.

    Args:
        setpoint       (float): target depth in metres
        pv             (float): current depth in metres (process variable)
        kp             (float): proportional gain
        ki             (float): integral gain
        kd             (float): derivative gain
        previous_error (float): error from the last loop iteration
        integral       (float): running integral accumulator
        dt             (float): time since last iteration in seconds

    Returns:
        tuple: (control, error, integral)
            control  — output value sent to set_syringe()
            error    — current error (setpoint – pv)
            integral — updated integral accumulator
    """
    error      = setpoint - pv
    integral  += error * dt
    derivative = (error - previous_error) / dt
    control    = kp * error + ki * integral + kd * derivative
    return control, error, integral


#──── Safety Cieling ──────────────────────────────────────────────────────────────────


SURFACE_CEILING = 0.40   # metres — hard limit, competition rules: –5 pts if broken


def safety_check(depth):
    """
    Hard safety override — runs before every PID step.

    If the float is shallower than SURFACE_CEILING, immediately force
    full extension (sink). Also resets the PID integrator so accumulated
    integral windup cannot push back through the ceiling.

    Returns:
        bool: True if override fired (caller should skip PID this tick)
    """
    if depth < SURFACE_CEILING:
        print(f"  SAFETY OVERRIDE: {depth:.3f} m < {SURFACE_CEILING} m "
              f"— forcing descent")
        set_syringe(100)
        return True
    return False



#───── Data Logging — writes to CSV on Pi, read after recovery────────────────────────────
#       Description: Data is saved to ~/Mate-Rov/run_<timestamp>.csv

import csv

log        = []              # full record: (timestamp, depth, profile_number)
live_times  = deque(maxlen=500)
live_depths = deque(maxlen=500)

# Create output folder and file once at import time
_data_dir  = "/home/programming-pathway/float_data"
os.makedirs(_data_dir, exist_ok=True)
_run_label = time.strftime("%Y%m%d_%H%M%S")
CSV_PATH   = os.path.join(_data_dir, f"run_{_run_label}.csv")

# Write CSV header
with open(CSV_PATH, "w", newline="") as f:
    csv.writer(f).writerow(["timestamp_s", "elapsed_s", "depth_m", "profile"])

_log_start = None   # set on first point so elapsed_s starts at 0


def log_point(depth, profile_num):
    """
    Record a depth reading to the in-memory log and append to CSV immediately.

    Writing each row as it arrives means data is safe even if the Pi loses
    power or crashes mid-dive — no data held only in RAM.

    Args:
        depth       (float): current depth in metres
        profile_num (int)  : 1 or 2
    """
    global _log_start
    t = time.time()

    if _log_start is None:
        _log_start = t
    elapsed = round(t - _log_start, 2)

    log.append((t, depth, profile_num))
    live_times.append(t)
    live_depths.append(depth)

    # Append row to CSV immediately — safe against power loss
    with open(CSV_PATH, "a", newline="") as f:
        csv.writer(f).writerow([round(t, 2), elapsed, round(depth, 3), profile_num])

    print(f"  [LOG] elapsed={elapsed:.2f}s  depth={round(depth,3)}m  profile={profile_num}")


#──── Graph Functions────────────────────────────────────────────────────────────────────────────

plt.ion()
fig_live, ax_live = plt.subplots(figsize=(9, 4))
ax_live.set_title("Live depth profile")
ax_live.set_xlabel("Time (s)")
ax_live.set_ylabel("Depth (m)")
ax_live.invert_yaxis()
ax_live.axhline(2.5,  color="steelblue", linestyle="--",
                linewidth=0.8, label="2.5 m target")
ax_live.axhline(0.40, color="red",       linestyle="--",
                linewidth=0.8, label="40 cm ceiling")
ax_live.legend(loc="upper right")
live_line, = ax_live.plot([], [], color="steelblue", linewidth=1.8)
fig_live.tight_layout()
fig_live.show()

_live_start = None   # set on first data point so x-axis starts at 0


def update_live_plot():
    """Refresh the live depth chart. Called once per control loop tick."""
    global _live_start
    if not live_times:
        return
    if _live_start is None:
        _live_start = live_times[0]
    xs = [t - _live_start for t in live_times]
    ys = list(live_depths)
    live_line.set_data(xs, ys)
    ax_live.relim()
    ax_live.autoscale_view()
    fig_live.canvas.draw()
    fig_live.canvas.flush_events()


#   Final Graph (after both profiles complete)

def plot_final():
    if not log:
        print("No data to plot"); return

    start_t = log[0][0]
    colors  = {1: "steelblue", 2: "coral"}

    fig, ax = plt.subplots(figsize=(11, 5))

    for profile_num in [1, 2]:
        pts = [(t - start_t, d) for t, d, p in log if p == profile_num]
        if not pts:
            continue
        times, depths = zip(*pts)
        ax.plot(times, depths,
                label=f"Profile {profile_num}",
                color=colors[profile_num],
                linewidth=1.8)

    # Target lines
    ax.axhline(2.5,  color="gray", linestyle="--", linewidth=0.8,
               label="2.5 m target")
    ax.axhline(0.40, color="red",  linestyle="--", linewidth=0.8,
               label="40 cm ceiling")

    # Shaded hold zones
    ax.axhspan(2.4,  2.6,  alpha=0.08, color="steelblue")
    ax.axhspan(0.35, 0.45, alpha=0.08, color="red")

    ax.invert_yaxis()
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Depth (m)")
    ax.set_title("Vertical profiles — depth over time")
    ax.legend()
    fig.tight_layout()

    out = os.path.join(_data_dir, f"graph_{_run_label}.png")
    fig.savefig(out, dpi=150)
    print(f"Final graph saved → {out}")

    plt.ioff()
    plt.show()


#   run profile
def run_profile(profile_num, kp, ki, kd):
    """
    Execute one full vertical profile.

    Phase 1: Descend to 2.5 m, hold for 30 s
    Phase 2: Ascend to 40 cm,  hold for 30 s

    Safety ceiling is enforced on every tick regardless of phase.
    Measured dt is used for PID (never assumed) so timing stays accurate
    even if sensor reads or plot updates take variable time.

    """
    phases = [
        {"label": "Descend to 2.5 m", "setpoint": 2.50, "duration": 30},
        {"label": "Hold at 40 cm",     "setpoint": 0.40, "duration": 30},
    ]

    previous_error = 0.0
    integral       = 0.0

    print(f"\n{'='*48}")
    print(f"  Profile {profile_num} starting")
    print(f"{'='*48}")
    print(f"  [LOG] profile={profile_num} started at {time.strftime('%H:%M:%S')}")

    for phase in phases:
        setpoint = phase["setpoint"]
        end_time = time.time() + phase["duration"]
        print(f"\n  ── {phase['label']} ──")

        while time.time() < end_time:
            loop_start = time.time()

            # Read sensor
            try:
                depth = depth_m()
            except RuntimeError as e:
                print(f"  Sensor error: {e} — retrying")
                time.sleep(0.05)
                continue

            log_point(depth, profile_num)
            update_live_plot()

            # Safety ceiling — must run before PID every tick
            if safety_check(depth):
                integral       = 0.0   # reset to kill windup
                previous_error = 0.0
                dt = time.time() - loop_start
                time.sleep(max(0.0, 0.1 - dt))
                continue

            # Measure real dt — never assume it
            dt = max(0.01, time.time() - loop_start)

            control, previous_error, integral = pid_controller(
                setpoint, depth, kp, ki, kd, previous_error, integral, dt
            )

            # PID output drives syringe directly
            # +control = need to go deeper  → extend → sink
            # –control = need to rise        → retract → float up
            set_syringe(control)

            print(f"  depth={depth:.3f} m  target={setpoint} m  "
                  f"err={previous_error:+.3f}  ctrl={control:+.2f}")

            # Sleep the remainder of the 100 ms budget
            dt = time.time() - loop_start
            time.sleep(max(0.0, 0.1 - dt))

    syringe_stop()
    print(f"  [LOG] profile={profile_num} ended at {time.strftime('%H:%M:%S')}")
    print(f"\n  Profile {profile_num} complete")


# ── main ────────────────────────────────────────────────────────────────

def main():
    global surface_pressure

    # ── Sensor init ──────────────────────────────────────────────────────────
    sensor.setFluidDensity(1000)   # pool water (fresh) kg/m³
                                   # swap to ms5837.DENSITY_SALTWATER for ocean

    if not sensor.init():
        print("ERROR: Sensor init failed — check wiring on pins 3 and 5")
        return
    if not sensor.read():
        print("ERROR: First sensor read failed")
        return

    # Lock in surface pressure before submerging
    surface_pressure = sensor.pressure(ms5837.UNITS_Pa)
    print(f"Surface pressure locked: {surface_pressure:.1f} Pa — data will be saved to {CSV_PATH}")

    # ── PID gains (tune these for your robot) ────────────────────────────────
    # Start with kp only (ki=0, kd=0), add kd to reduce overshoot,
    # then add small ki to remove any steady-state depth error.
    kp, ki, kd = 1.0, 0.0, 0.0

    # ── Known start position: fully retracted (floating) ─────────────────────
    set_syringe(-100)
    time.sleep(3)         # give syringe time to fully travel
    syringe_stop()

    # ── Two vertical profiles ─────────────────────────────────────────────────
    run_profile(1, kp, ki, kd)

    print("\n  Surface pause between profiles (5 s)...")
    set_syringe(-100)     # retract back to surface float position
    time.sleep(3)
    syringe_stop()
    time.sleep(2)

    run_profile(2, kp, ki, kd)

    print(f"\nTotal data points logged: {len(log)}")
    print(f"CSV saved to: {CSV_PATH}")

    plot_final()

    gpio_cleanup()
    print("\nDone.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted — cleaning up")
        gpio_cleanup()
