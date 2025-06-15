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
python3 processing_node_yolo.py'
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
rosrun base final_subscriber_node_line.py
```

