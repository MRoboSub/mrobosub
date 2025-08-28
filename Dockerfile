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

SHELL ["/bin/bash", "-c"] 

# Create workspace structure
RUN mkdir -p /root/ros2_ws/src && \
    cd /root/ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install && \
    source /root/ros2_ws/install/setup.bash && \
    rosdep install --from-paths /root/ros2_ws/src -y --ignore-src

# Copy dotfiles
COPY .vimrc /root/

WORKDIR /root/ros2_ws/src/

EXPOSE 10000

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc && \
    ln -s "/root/ros2_ws/src/.bash_aliases" "/root/.bash_aliases"

