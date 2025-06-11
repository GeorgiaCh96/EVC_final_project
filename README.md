# How to achieve communication between docker container and the jetson.

These instructions are applicable for Windows machines.

First of all, open the following applications:


* XLaunch:
  1. Run XLaunch from the Windows Taskbar
  2. Select Multiple Windows
  3. Set Display number to 0
  4. Select Start no client
  5. Check Disable access control
  6. Finish

* Docker Desktop

Also, you have to disable the public network firewall because it will try to block UDP communications between your container and the 
Jetson


In this demo, we work with the Workshop4 scripts, and we will run the **processing node** on the docker container, while the
**publisher** and **subscriber nodes** will run on the jetson.

In a nutshell:

* The **camera publisher node** is publishing raw images. 

* The **camera processing node** is  subscribing in raw images. It publishes:
  1.	processed images (images went through fisheye view undistortion method, face detection and facial emotion recognition models), topic name: **/camera/processed_image**
  2.	facial emotion recognition predictions, topic name: **/camera/FER_prediction**

* Finally, the **camera subscriber node** is subscribing for processed images and FER predictions.

The ROS nodes communication is described by the following RQT graph:

![rqt_graph](../workshop4_2074028_dockerized_LOCAL/Screenshot 2025-05-22 185013.png)

First, in the **processing_node** you have to set fixed ports:

`rospy.init_node('processing_node', xmlrpc_port=45100, tcpros_port=45101)`

You're telling the processing_node (running inside Docker) — to:

* Accept XML-RPC requests (e.g., for parameter server, node registration) on port 45100

* Accept TCPROS connections (e.g., for topic communication -- pub/sub) on port 45101

If you don’t define the ports manually, then ROS chooses random ones — which will not
work across machines.

-----------------------------------------
DOCKER
------

First build the container:

`docker build -t test_node:latest .`


To interact with the container you have to explicitly set the ROS_MASTER_URI to the jetson's IP, and ROS_IP to your 
PC's IPv4 address. Also you have to 
expose the XML-RPC and TCPROS ports that you defined in the rospy.init_node.


`docker run -it --rm -e ROS_MASTER_URI=http://192.168.8.2:11311 -e ROS_IP=192.168.8.141 -p 45100:45100 -p 45101:45101 test_node
`


Inside the docker set these environment variables:

`export ROS_MASTER_URI=http://192.168.8.2:11311`

`export ROS_IP=192.168.8.141 `

`export DISPLAY=192.168.8.141:0`

Then run the following commands: 

#### Step 1. change ownership
`sudo chown -R ubuntu:ubuntu /home/ubuntu/traffic_ws`


#### Step 2: Clean the previous build (optional, but good after big changes)
`sudo rm -rf build/ devel/`

#### Step 3: Source ROS (required for catkin_make)
`source /opt/ros/noetic/setup.bash`


#### Step 4: Rebuild your workspace
`catkin_make`

#### Step 5: Source your workspace (to expose messages, services, etc.)
`source devel/setup.bash`

#### Step 6 (optional): Add PYTHONPATH to .bashrc if not already added
`echo 'export PYTHONPATH=$PYTHONPATH:/home/ubuntu/traffic_ws/devel/lib/python3.8/dist-packages' >> ~/.bashrc`

#### Step 7 (optional): Add workspace sourcing to .bashrc if not already added
`echo 'source ~/traffic_ws/devel/setup.bash' >> ~/.bashrc`

#### Step 8 (only needed if you modified .bashrc during this session)

`source ~/.bashrc`

Then run the node

'python src/motion/src/camera_processing_node.py'

`cd src/jetson_camera/src/`

`python camera_processing_node.py`




------------------------------------------
JETSON
-----

First, check if `xeyes` works. 
If it does not work, then run on your local machine:

`ipconfig` and evaluate your PC's IPv4 Address. 

e.g. `IPv4 ...... 192.168.8.248`

Then run:

export `DISPLAY=192.168.8.248:0`

Check again if `xeyes` works. 
Then, navigate in your workspace, e.g.

`cd ~/EVC/workshops/Georgia_docker/workshop4_motion_2074028`

and run:

`catkin_make`

`source ~/.bashrc`

`source ./devel/setup.bash`

Note: you might also need to run

`export DISPLAY=192.168.8.248:0
`
with the IP pointing to your PC's IPv4. If you do that, then run again:

`source ~/.bashrc`


To launch the ROS publisher and subscriber nodes, run:

`roslaunch jetson_camera camera_publisher.launch`

`roslaunch jetson_camera camera_subscriber.launch`



