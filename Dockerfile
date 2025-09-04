FROM docker.io/osrf/ros:jazzy-desktop

RUN apt-get update && \
    apt-get install -y  python-is-python3 \
                        python3-pip \
                        git \
                        vim \
                        screen \
                        python3-tk \
                        libudev-dev \
                        mypy \
                        tmux \
                        less \
                        ros-jazzy-ros2-control \
                        ros-jazzy-ros2-controllers \
                        python3-typing-extensions \
                        python3-scipy \
                        python3-transforms3d

RUN echo "ALL ALL = (ALL) NOPASSWD: ALL" >> /etc/sudoers

SHELL ["/bin/bash", "-c"] 

USER 1000:1000

RUN mkdir -p /home/ubuntu/jlb_pid_ws/src && \
    cd /home/ubuntu/jlb_pid_ws/src && \
    git clone https://github.com/HenryLeC/ros2-pid.git && \
    cd /home/ubuntu/jlb_pid_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install

# Create workspace structure
RUN mkdir -p /home/ubuntu/ros2_ws/src && \
    cd /home/ubuntu/ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install && \
    source /home/ubuntu/ros2_ws/install/setup.bash
#    rosdep install --from-paths /home/ubuntu/ros2_ws/src -y --ignore-src

# Copy dotfiles
COPY .vimrc /root/

WORKDIR /home/ubuntu/ros2_ws/src/

EXPOSE 10000

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "source /home/ubuntu/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "source /home/ubuntu/jlb_pid_ws/install/setup.bash" >> /home/ubuntu/.bashrc && \
    ln -s "/home/ubuntu/ros2_ws/src/.bash_aliases" "/home/ubuntu/.bash_aliases"

