import lgpio
import time
import sys

GPIO_PIN = 13 #set value to GPIO pin connected to actuator
PWM_FREQ = 50 #Hz - standard servo frequency

#Actuator Pulse Range
PULSE_MIN = 1050 #Fully retracted (0 mm)
PULSE_MAX = 1950

def connect():
    """Open GPIO chip handle."""
    try:
        h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(h, GPIO_PIN)
        return h
    except Exception as e:
        sys.exit(f"ERROR: Could not open GPIO: {e}\nTry running with sudo.")

def set_position(h, pulse_us: int) -> None:
    """
    Move actuator to a position specified by pulse width in microseconds.
    pulse_us must be between PULSE_MIN (retracted) and PULSE_MAX (extended).
    """
    pulse_us = max(PULSE_MIN, min(PULSE_MAX, pulse_us))
    lgpio.tx_servo(h, GPIO_PIN, pulse_us, PWM_FREQ)


def position_to_pulse(position_mm: float) -> int:
    """
    Convert a position in mm (0–100) to a PWM pulse width in µs.
    0 mm   = fully retracted = PULSE_MIN
    100 mm = fully extended  = PULSE_MAX
    """
    position_mm = max(0.0, min(100.0, position_mm))
    fraction = position_mm / 100.0
    return int(PULSE_MIN + fraction * (PULSE_MAX - PULSE_MIN))


def move_to_mm(h, position_mm: float) -> None:
    """Move actuator to an absolute position in millimetres (0–100)."""
    pulse = position_to_pulse(position_mm)
    print(f"  Moving to {position_mm:.1f} mm  ({pulse} µs)")
    set_position(h, pulse)


def sweep(h, start_mm: float, end_mm: float,
          step_mm: float = 5.0, delay_s: float = 0.3) -> None:
    """Sweep the actuator from start_mm to end_mm in step_mm increments."""
    direction = 1 if end_mm >= start_mm else -1
    pos = start_mm
    while (direction == 1 and pos <= end_mm) or (direction == -1 and pos >= end_mm):
        move_to_mm(h, pos)
        time.sleep(delay_s)
        pos += direction * step_mm


def release(h) -> None:
    """Stop sending PWM — actuator holds last position."""
    lgpio.tx_servo(h, GPIO_PIN, 0)
    print("  PWM signal released (actuator holds position).")


# ── Demo / main ───────────────────────────────────────────────────────────────
def main():
    h = connect()
    print(f"lgpio connected. Controlling actuator on GPIO {GPIO_PIN}.\n")

    try:
        print("1. Moving to fully retracted (0 mm)...")
        move_to_mm(h, 0)
        time.sleep(2)

        print("2. Moving to midpoint (100 mm)...")
        move_to_mm(h, 100)
        time.sleep(5)

        print("3. Moving to fully extended (0 mm)...")
        move_to_mm(h, 0)
        time.sleep(5)

        print("4. Sweeping back to 0 mm in 5 mm steps...")
        sweep(h, 100, 0, step_mm=5, delay_s=0.25)

        print("\nDemo complete.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    finally:
        release(h)
        lgpio.gpiochip_close(h)
        print("GPIO closed.")


if __name__ == "__main__":
    main()
