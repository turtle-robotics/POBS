"""
Run 4 Blue Robotics T200 thrusters via Adafruit 16-Channel PWM/Servo HAT.

Wiring: T200 ESC signal wires → HAT channels 0–3
        ESC power from external supply (NOT the Pi)

Install deps:
    pip install adafruit-circuitpython-servokit
"""

import time
import board
import busio
from adafruit_servokit import ServoKit

# --- Constants -----------------------------------------------------------
NUM_MOTORS      = 4
MOTOR_CHANNELS  = [0, 1, 2, 3]   # HAT PWM channels used

# T200 / Basic ESC pulse widths (microseconds)
PWM_NEUTRAL     = 1500            # stopped
PWM_FULL_FWD    = 1900            # 100 % forward
PWM_FULL_REV    = 1100            # 100 % reverse

# Target throttle (positive = forward)
TARGET_THROTTLE = 0.15            # 15 %

# Ramp parameters
RAMP_STEPS      = 20              # number of increments to reach target
RAMP_DELAY_S    = 0.1             # seconds between each step


def throttle_to_us(throttle: float) -> int:
    """
    Convert throttle fraction [-1.0, 1.0] to a PWM pulse width in µs.
    0.0  → 1500 µs (neutral)
    1.0  → 1900 µs (full forward)
    -1.0 → 1100 µs (full reverse)
    """
    throttle = max(-1.0, min(1.0, throttle))
    if throttle >= 0:
        return int(PWM_NEUTRAL + throttle * (PWM_FULL_FWD - PWM_NEUTRAL))
    else:
        return int(PWM_NEUTRAL + throttle * (PWM_NEUTRAL - PWM_FULL_REV))


def set_motors(kit: ServoKit, channels: list, pulse_us: int) -> None:
    """Send the same pulse width (µs) to all motor channels."""
    for ch in channels:
        kit.servo[ch].set_pulse_width_range(PWM_FULL_REV, PWM_FULL_FWD)
        # ServoKit angle maps linearly: 0° → min pulse, 180° → max pulse
        # Neutral (1500 µs) sits at 90°; convert pulse_us to angle.
        angle = (pulse_us - PWM_FULL_REV) / (PWM_FULL_FWD - PWM_FULL_REV) * 180.0
        kit.servo[ch].angle = angle


def arm_escs(kit: ServoKit, channels: list, arm_time_s: float = 3.0) -> None:
    """
    Send neutral signal to arm the ESCs.
    Blue Robotics Basic ESC requires neutral at power-on before it will respond.
    """
    print("Arming ESCs — sending neutral (1500 µs) …")
    set_motors(kit, channels, PWM_NEUTRAL)
    for remaining in range(int(arm_time_s), 0, -1):
        print(f"  {remaining}s …")
        time.sleep(1.0)
    print("ESCs armed.\n")


def ramp_to(kit: ServoKit, channels: list,
            start_throttle: float, end_throttle: float) -> None:
    """Gradually move from start_throttle to end_throttle."""
    for step in range(1, RAMP_STEPS + 1):
        t = start_throttle + (end_throttle - start_throttle) * (step / RAMP_STEPS)
        pulse = throttle_to_us(t)
        set_motors(kit, channels, pulse)
        print(f"  throttle {t*100:5.1f}%  →  {pulse} µs")
        time.sleep(RAMP_DELAY_S)


def main() -> None:
    # Initialise the HAT at 50 Hz (standard RC/ESC frequency)
    i2c = busio.I2C(board.SCL, board.SDA)
    kit = ServoKit(channels=16, i2c=i2c, frequency=50)

    try:
        arm_escs(kit, MOTOR_CHANNELS)

        print(f"Ramping up to {TARGET_THROTTLE*100:.0f}% throttle …")
        ramp_to(kit, MOTOR_CHANNELS, 0.0, TARGET_THROTTLE)
        print(f"\nAll {NUM_MOTORS} motors running at {TARGET_THROTTLE*100:.0f}% throttle.")
        print("Press Ctrl+C to stop.\n")

        while True:
            time.sleep(1.0)

    except KeyboardInterrupt:
        print("\nShutdown requested — ramping down …")
        ramp_to(kit, MOTOR_CHANNELS, TARGET_THROTTLE, 0.0)
        set_motors(kit, MOTOR_CHANNELS, PWM_NEUTRAL)
        print("Motors stopped.")


if __name__ == "__main__":
    main()
