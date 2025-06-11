import time
import math
import yaml
from motorDriver import DaguWheelsDriver
from encoderDriver import WheelEncoderDriver

# === Load calibration constants ===
with open("/home/jetbot/EVC/workshops/workshop4_motion/src/motion/src/motorStuffcalibration/calibration.yaml", "r") as f:
    calib = yaml.safe_load(f)

WHEEL_RADIUS = calib["wheel_radius"]
WHEEL_BASELINE = calib["wheel_baseline"]
TICKS_PER_REV = calib["ticks_per_rev"]
GAIN = calib["gain"]
TRIM = calib["trim"]

class MotionController:
    def __init__(self, gpio_left=12, gpio_right=35):
        self.motor_driver = DaguWheelsDriver()
        self.encoder_left = WheelEncoderDriver(gpio_left)
        self.encoder_right = WheelEncoderDriver(gpio_right)

    def stop(self):
        self.motor_driver.set_wheels_speed(0, 0)

    def distance_to_ticks(self, distance):
        revs = distance / (2 * math.pi * WHEEL_RADIUS)
        return revs * TICKS_PER_REV

    def _adjusted_speeds(self, base_speed):
        left = base_speed * GAIN * (1 + TRIM)
        right = base_speed * GAIN * (1 - TRIM)
        #left= base_speed
        #right = base_speed
        # Inverted trim logic (fixes flip in drive code)
        # TRIM > 0 right faster  left turn
        # TRIM < 0  right slower  straighter

        # left = base_speed * GAIN * (1 - TRIM)
        # right = base_speed * GAIN * (1 + TRIM)

        return left, right

    def move_forward(self, distance_meters, speed=0.3):
        target_ticks = self.distance_to_ticks(distance_meters)
        start_left = self.encoder_left._ticks
        start_right = self.encoder_right._ticks
        left, right = self._adjusted_speeds(speed)

        self.motor_driver.set_wheels_speed(left, right)

        while True:
            left_ticks = self.encoder_left._ticks - start_left
            right_ticks = self.encoder_right._ticks - start_right
            avg_ticks = (left_ticks + right_ticks) / 2.0

            # Optional: debug prints
            #print("Ticks (L/R):", left_ticks, right_ticks)

            if avg_ticks >= target_ticks:
                break
            time.sleep(0.01)

        self.stop()

    def turn(self, angle_degrees, speed=0.3):
        """
        Turns the robot in place by the given angle (in degrees).
        Positive = right turn, Negative = left turn.
        """
        angle_rad = math.radians(angle_degrees)

        # Arc length each wheel must travel (in meters)
        arc = (WHEEL_BASELINE * abs(angle_rad)) / 2

        # Convert arc to encoder ticks
        target_ticks = self.distance_to_ticks(arc)
        print("Expected ticks per wheel: {:.1f}".format(target_ticks))

        # Start encoder values
        start_left = self.encoder_left._ticks
        start_right = self.encoder_right._ticks

        # Use only positive speed internally
        abs_speed = abs(speed)
        left_cmd, right_cmd = self._adjusted_speeds(abs_speed)

        # Set direction based on turn direction
        if angle_degrees > 0:
            # Right turn: left wheel forward, right wheel backward
            self.motor_driver.set_wheels_speed(left_cmd, -right_cmd)
            print("Turning RIGHT: L=+{:.2f}, R=-{:.2f}".format(left_cmd, right_cmd))
        else:
            # Left turn: left wheel backward, right wheel forward
            self.motor_driver.set_wheels_speed(-left_cmd, right_cmd)
            print("Turning LEFT: L=-{:.2f}, R=+{:.2f}".format(left_cmd, right_cmd))

        # Wait for both wheels to hit target ticks
        while True:
            dl = abs(self.encoder_left._ticks - start_left)
            dr = abs(self.encoder_right._ticks - start_right)
            avg = (dl + dr) / 2.0

            if avg >= target_ticks:
                break
            time.sleep(0.01)

        self.stop()

        # Final result print
        print("Actual ticks L/R: {}, {}".format(dl, dr))

