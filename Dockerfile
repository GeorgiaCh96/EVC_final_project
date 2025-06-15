# Base image with ROS noetic (Python3.8 default)
FROM osrf/ros:noetic-desktop

# Create non-root user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME

#RUN rm /etc/apt/sources.list.d/ros1-latest.list

# Install base packages
RUN apt-get update && apt-get install -y \
    git iputils-ping x11-apps sshfs sshpass net-tools \
    netcat openssh-server avahi-daemon libnss-mdns iproute2 \
    tmux vim nano curl libzbar0 libzbar-dev


# Add sudo and Python 3
RUN apt-get update && \
    apt-get install -y sudo python3-dev python3-pip python3-tk && \
    # optional: make python3 the default
    update-alternatives --install /usr/bin/python python /usr/bin/python3 1 && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

RUN pip3 install --no-cache-dir \
      torch==1.8.1+cpu \
      torchvision==0.9.1+cpu \
      torchaudio==0.8.1 \
      -f https://download.pytorch.org/whl/torch_stable.html

COPY requirements-mediapipe.txt /requirements-mediapipe.txt

RUN pip3 install --no-cache-dir -r /requirements-mediapipe.txt

COPY traffic_ws/src/base/requirements.txt /requirements.txt
RUN pip3 install --no-cache-dir -r /requirements.txt

COPY traffic_ws/src/base/dependencies.txt /dependencies.txt
RUN pip3 install --no-cache-dir -r /dependencies.txt

COPY requirements-color.txt /requirements-color.txt

RUN pip3 install --no-cache-dir -r /requirements-color.txt

RUN pip3 install --no-cache-dir ultralytics

COPY traffic_ws /home/ubuntu/traffic_ws

RUN rm -rf /home/ubuntu/traffic_ws/build /home/ubuntu/traffic_ws/devel /home/ubuntu/traffic_ws/install && \
    chown -R ubuntu:ubuntu /home/ubuntu/traffic_ws

COPY traffic_ws/src/startup.sh /home/ubuntu/traffic_ws/src/startup.sh
RUN chmod +x /home/ubuntu/traffic_ws/src/startup.sh && chown ubuntu:ubuntu /home/ubuntu/traffic_ws/src/startup.sh


# Rosdep update
RUN rosdep update

# Set environment variables in user bashrc (ROS networking config)
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
# ros_master_uri = jetson's IP
ARG ROS_MASTER_URI=http://192.168.8.2:11311
# client_ip = my laptops IP
ARG CLIENT_IP=192.168.8.141 
ARG DISPLAY_VAR=:0
ENV DISPLAY=${DISPLAY_VAR}

RUN echo "export ROS_MASTER_URI=$ROS_MASTER_URI" >> ~/.bashrc && \
    echo "export ROS_IP=$CLIENT_IP" >>  ~/.bashrc && \
    echo "export DISPLAY=$CLIENT_IP:0"  >> ~/.bashrc


# Copy project
ARG HOME_DIR=/home/$USERNAME/

RUN echo "export PYTHONPATH=$PYTHONPATH:/home/ubuntu/traffic_ws/devel/lib/python3.8/dist-packages" >> /home/ubuntu/.bashrc

# Add user to video group
RUN usermod --append --groups video $USERNAME

# Switch to non-root user
USER $USERNAME

WORKDIR /home/$USERNAME/traffic_ws

CMD ["bash", "-c", "/home/ubuntu/traffic_ws/src/startup.sh"]
