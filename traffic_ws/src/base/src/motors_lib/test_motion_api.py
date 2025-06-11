from motion_api import MotionController
import time

# Create instance
motion = MotionController()

# === Test 1: Move forward 0.5 meters ===
print("Moving forward 0.5 meters...")
motion.move_forward(0.1, speed=0.2)
time.sleep(5)

# === Test 2: Turn 90 degrees right ===
print("Turning 90 degrees right...")
motion.turn(90, speed=0.3)
time.sleep(5)

# === Test 3: Turn 90 degrees left ===
print("Turning 90 degrees left...")
motion.turn(-90, speed=0.3)
time.sleep(1)

# Done
motion.stop()
print("Test finished.")
