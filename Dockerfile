FROM docker.io/osrf/ros:humble-desktop
# MacOS users with Apple Silicon should use the line below instead:
# FROM --platform=linux/arm64 docker.io/osrf/ros:humble-desktop

RUN apt-get update && \
    apt-get install -y  python-is-python3 \
                        python3-pip \
                        git \
                        vim \
                        screen \
                        python3-tk \
                        libudev-dev \
                        tmux \
                        less \
                        ros-humble-ros2-control \
                        ros-humble-ros2-controllers \
                        python3-typing-extensions \
                        python3-scipy \
                        python3-transforms3d \
                        python3-serial \
                        pipx \
                        libusb-dev

RUN echo "ALL ALL = (ALL) NOPASSWD: ALL" >> /etc/sudoers && \
    useradd -m -s /bin/bash -G sudo ubuntu && \
    echo "ubuntu:ubuntu" | chpasswd

SHELL ["/bin/bash", "-c"] 

USER 1000:1000

RUN pipx install mypy && \
    pipx ensurepath

RUN mkdir -p /home/ubuntu/jlb_pid_ws/src && \
    cd /home/ubuntu/jlb_pid_ws/src && \
    git clone https://github.com/HenryLeC/ros2-pid.git && \
    cd /home/ubuntu/jlb_pid_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install
    
# Create workspace structure
RUN mkdir -p /home/ubuntu/ros2_ws/src && \
    cd /home/ubuntu/ros2_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install && \
    source /home/ubuntu/ros2_ws/install/setup.bash
#    rosdep install --from-paths /home/ubuntu/ros2_ws/src -y --ignore-src

# Copy dotfiles
COPY .vimrc /root/

WORKDIR /home/ubuntu/ros2_ws/src/

EXPOSE 10000

# Add coloring to ros messages
RUN echo "export RCUTILS_COLORIZED_OUTPUT=1" >> /home/ubuntu/.bashrc

RUN echo "source /opt/ros/humble/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "source /home/ubuntu/jlb_pid_ws/install/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "source /home/ubuntu/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc && \
    ln -s "/home/ubuntu/ros2_ws/src/.bash_aliases" "/home/ubuntu/.bash_aliases"

