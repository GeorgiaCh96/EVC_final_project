import time
import csv
import yaml



from motorDriver import DaguWheelsDriver
from encoderDriver import *

# === Constants ===
TICKS_PER_REV = 137
GPIO_LEFT = 12 #gpio 18
GPIO_RIGHT = 35 #gpio19
DURATION = 5  # seconds
SPEEDS =[-0.6, 0.6 ]  # speed test range

# === Measure motor performance ===
def calibrate_motor(name, gpio_pin, is_left=True, duration=5, speeds=None):
    if speeds is None:
        speeds = SPEEDS

    motor = DaguWheelsDriver()
    encoder = WheelEncoderDriver(gpio_pin)
    log_file = "{}_motor_calibration.csv".format(name)

    with open(log_file, mode='w') as f:
        writer = csv.writer(f)
        writer.writerow(["Speed", "Ticks", "Ticks/sec", "Angular velocity (rad/s)"])

        for speed in speeds:
            print("\nTesting {} motor at speed: {}".format(name, speed))

            if is_left:
                motor.set_wheels_speed(left=speed, right=0)
            else:
                motor.set_wheels_speed(left=0, right=speed)

            start_ticks = encoder._ticks
            print("start ticks: {}".format(start_ticks))
            time.sleep(duration)
            end_ticks = encoder._ticks
            print("end ticks: {}".format(end_ticks))
            motor.set_wheels_speed(0, 0)

            delta_ticks = end_ticks - start_ticks
            ticks_per_sec = delta_ticks / float(duration)
            angular_velocity = (delta_ticks * 2 * 3.1416) / (TICKS_PER_REV * float(duration))

            print("Speed: {}, Deltaticks: {}, omega: {:.3f} rad/s".format(speed, delta_ticks, angular_velocity))
            writer.writerow([speed, delta_ticks, ticks_per_sec, angular_velocity])
            time.sleep(1)

    print("\nFinished {} motor. Logged to: {}".format(name, log_file))
    motor.set_wheels_speed(0, 0)
    return angular_velocity  # optionally return for gain/trim use

# === Save calibration values to YAML ===
def save_to_yaml(filename, data):
    with open(filename, "w") as f:
        yaml.dump(data, f)
    print("\nSaved calibration to: {}".format(filename))

# === MAIN ===
if __name__ == "__main__":
    print("==== MOTOR CALIBRATION START ====")

    # Run left motor calibration
    omega_left = calibrate_motor("left", GPIO_LEFT, is_left=True)

    # Run right motor calibration
    omega_right = calibrate_motor("right", GPIO_RIGHT, is_left=False)

    # Compute gain and trim using final test
    print("\nLeft omega: {:.3f} rad/s, Right omega: {:.3f} rad/s".format(omega_left, omega_right))

    if omega_left + omega_right == 0:
        trim = 0
        print("trim set to 0")
    else:
        trim = (omega_right - omega_left) / (omega_right + omega_left)

    gain = (omega_left + omega_right) / 2.0


    print("\nGain (g): {:.3f}".format(gain))
    print("Trim (t): {:.3f}".format(trim))

    # Save calibration constants
    calibration_data = {
        "wheel_radius": 0.032,
        "wheel_baseline": 0.15, #speed 0.01
        "ticks_per_rev": TICKS_PER_REV,
        "gain": round(gain, 3),
        "trim": round(trim, 3)
    }

    save_to_yaml("calibration.yaml", calibration_data)
    print("==== MOTOR CALIBRATION COMPLETE ====")
