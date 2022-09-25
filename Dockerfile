FROM osrf/ros:noetic-desktop

# turtlebot3 packages, vim, screen
RUN apt-get update && \
    apt-get install -y  python3-catkin-tools \
                        git \
                        vim \
                        screen

SHELL ["/bin/bash", "-c"] 

# Create workspace structure
RUN mkdir -p /root/catkin_ws/src && \
    cd /root/catkin_ws && \
    source /opt/ros/noetic/setup.bash && \
    catkin build

# Unity simulation
RUN cd /root/catkin_ws/src && \
    git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint

# Build new packages
RUN cd /root/catkin_ws && \
    source /root/catkin_ws/devel/setup.bash && \
    catkin build

# Copy dotfiles
COPY .vimrc /root/
COPY .screenrc /root/

WORKDIR /root/catkin_ws

EXPOSE 10000

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc
