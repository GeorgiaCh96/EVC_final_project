# Traffic Robot System — Setup Instructions (Windows)

These instructions will guide you through setting up and running the traffic robot control system using Docker, ROS, and supporting scripts. This guide is tailored specifically for **Windows environments**.

---
## Prerequisites: Start the Required Applications

Before running the system, make sure the following services are configured and running:

### XLaunch (X11 Forwarding)

  1. Run XLaunch from the Windows Taskbar.
  2. Select Multiple Windows.
  3. Set Display number to 0.
  4. Select Start no client.
  5. Check Disable access control.
  6. Finish.

### Docker Desktop

Ensure Docker Desktop is installed and running.

### Disable Windows Public Network Firewall (Temporarily)

UDP communication between your container and the Jetson device might be blocked by the firewall. To avoid this temporarily **disable the Public Network firewall**.

## Running the Docker Container

To interact with the container you have to explicitly set the `ROS_MASTER_URI` to the jetson's IP, and `ROS_IP` to your 
PC's IPv4 address. These IP addresses must be correctly configured in two places:

At the `Dockerfile` locate and change:

```
ARG ROS_MASTER_URI=http://192.168.8.2:11311
ARG CLIENT_IP=192.168.8.141
```

At the `traffic_ws/src/startup.sh` locate and change:

```
export ROS_MASTER_URI=http://192.168.8.2:11311
export ROS_IP=192.168.8.141
export DISPLAY=192.168.8.141:0
```

 Then build the container:

`docker build -t test_node:latest .`

Start the container using the following command:


`docker run -it --rm -e ROS_MASTER_URI=http://192.168.8.2:11311 -e ROS_IP=192.168.8.141 -p 45100:45100 -p 45101:45101 -p 45200:45200 -p 45201:45201 -p 45300:45300 -p 45301:45301 -p 45400:45400 -p 45401:45401 -p 45500:45500 -p 45501:45501 -p 45600:45600 -p 45601:45601 -p 45700:45700 -p 45701:45701 test_node
`

## Launching the ROS nodes

Once the container is running, you need to launch the appropriate scripts on both:

- **The Docker container (Windows host)**
- **The Jetson device (ROS master)**

---

### On the Docker Container (Your Windows Machine)

Inside the container, navigate to the following directory:

```
cd ~/traffic_ws/src/base/src
```

Then, run the following Python scripts, each in a new terminal (or use `tmux`):

```
python3 calibration_node.py
python3 processing_node_line.py
python3 processing_node_color.py
python3 processing_node_barcode.py
python3 processing_node_gestures.py
python3 processing_node_motion_det.py
python3 processing_node_yolo.py
python3 decision_node.py
```

### On the Jetson (ROS Master)

After connection to the Jetson via SSH, launch the following nodes by:

Navigate to the workspace:

```
cd ~/traffic_ws
```

Build the workspace:

```
catkin_make
```

Source the workspace so that ROS can recognize your packages:
```
source devel/setup.bash
```

Launch the nodes:

```
roslaunch base publisher.launch
roslaunch base final_subscriber.launch
```

## Python Scripts Overview

This section provides a brief explanation of the function of each Python script.

#### `publisher_node.py`  
Captures real-time video frames from the Jetson Nano's CSI camera and publishes.

#### `calibration_node.py`  
Applies distortion correction to raw camera images using precomputed calibration matrices and publishes undistorted frames for downstream processing.

#### `processing_node_line.py`  
Processes undistorted images to detect and track lane lines, generating directional commands (`LEFT`, `RIGHT`, or `CENTERED`) to help the robot stay on course.

#### `processing_node_color.py`  
Analyzes dominant colors in the image using KNN and issues movement commands based on predefined color rules, such as green for "GO" and red for "STOP".

#### `processing_node_barcode.py`  
Scans and decodes QR/barcodes from the video feed to interpret directional commands like "TURN LEFT" or "TURN RIGHT".

#### `processing_node_gestures.py`  
Uses MediaPipe Hands to detect hand gestures and publishes a forward movement command when an open-hand (five fingers extended) gesture is recognized.

#### `processing_node_motion_det.py`  
Detects motion in the camera frame using background subtraction and contour analysis, locking or unlocking robot movement based on dynamic changes.

#### `processing_node_yolo.py`  
Applies a YOLOv8 object detection model to recognize road signs, triggering speed adjustments or stops when speed limit or STOP signs are detected.

#### `decision_node.py`  
Aggregates commands from all processing nodes and determines the robot’s next action based on a fixed-priority scheduling system to ensure safety and responsiveness.

#### `final_subscriber_node_line.py`  
Receives commands from the decision and line tracking nodes, translating them into motor driver signals to execute actions like moving forward, stopping, or turning.

## ROS Node Graph

The following diagram shows the communication between all ROS nodes using `rqt_graph`:
