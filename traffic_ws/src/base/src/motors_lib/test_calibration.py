from motion_api import MotionController
import time
import csv

motion = MotionController()

# === Settings ===
distance_m = 1
speed = 0.3
label = "forward_test"

# === Run ===
print("Running forward {:.2f} meters at speed {:.2f}".format(distance_m, speed))

# Get starting ticks
start_left = motion.encoder_left._ticks
start_right = motion.encoder_right._ticks

# Run motion
motion.move_forward(distance_m, speed)
#motion.turn(-90, speed)

# Get final ticks
end_left = motion.encoder_left._ticks
end_right = motion.encoder_right._ticks

delta_left = end_left - start_left
delta_right = end_right - start_right
avg = (delta_left + delta_right) / 2.0
expected = motion.distance_to_ticks(distance_m)

print("\n=== Encoder Result ===")
print("Left ticks:  ", delta_left)
print("Right ticks: ", delta_right)
print("Average:     ", avg)
print("Expected ticks: ", expected)

# # Save to CSV
# with open("{}_result.csv".format(label), "w") as f:
#     writer = csv.writer(f)
#     writer.writerow(["Wheel", "Ticks"])
#     writer.writerow(["Left", delta_left])
#     writer.writerow(["Right", delta_right])
#     writer.writerow(["Average", avg])
