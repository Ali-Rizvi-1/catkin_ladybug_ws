FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO noetic

# Install dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    gnupg2 \
    curl \
    lsb-release \
    sudo \
    perl \
    libterm-readline-perl-perl \
    dialog \
    debconf-utils \
    libxerces-c3.2 \
    libraw1394-11 \
    libc6 \
    libusb-1.0-0 \
    xsdcxx \
    wget \
    libavcodec58 \
    libavformat58 \
    libavutil56 \
    libswscale5 \
    libglu1-mesa \
    libomp5 \
    freeglut3 \
    freeglut3-dev \
    libglu1-mesa-dev \
    libopencv-dev \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS repositories and install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install -y \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-tf \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init

# Create ladybug user
RUN useradd -m -d /home/ladybug_user -s /bin/bash ladybug_user && \
    usermod -aG sudo ladybug_user && \
    echo "ladybug_user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Create necessary directories
RUN mkdir -p /dev/bus/usb /dev/raw1394 /ladybug_sdk && \
    chown -R ladybug_user:ladybug_user /ladybug_sdk

# Setup ROS environment for ladybug_user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ladybug_user/.bashrc

# Switch to ladybug user
USER ladybug_user
WORKDIR /home/ladybug_user

# Create catkin workspace
RUN mkdir -p ~/catkin_ws/src

CMD ["/bin/bash"]


# Prevous:
# FROM ubuntu:20.04

# ENV DEBIAN_FRONTEND=noninteractive

# # Install dependencies
# RUN apt-get update && apt-get install -y \
#     libxerces-c3.2 \
#     libraw1394-11 \
#     libc6 \
#     libusb-1.0-0 \
#     xsdcxx \
#     wget \
#     libavcodec58 \
#     libavformat58 \
#     libavutil56 \
#     libswscale5 \
#     libglu1-mesa \
#     libomp5 \
#     freeglut3 \
#     freeglut3-dev \
#     libglu1-mesa-dev \
#     sudo \
#     perl \
#     libterm-readline-perl-perl \
#     dialog \
#     debconf-utils \
#     && rm -rf /var/lib/apt/lists/*

# # Add these lines before ROS installation
# RUN apt-get update && apt-get install -y \
#     gnupg2 \
#     curl \
#     lsb-release \
#     build-essential \
#     cmake \
#     git

# # Now ROS installation
# RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
#     curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
#     apt-get update && \
#     apt-get install -y \
#     ros-noetic-ros-base \
#     python3-rosdep \
#     python3-rosinstall \
#     python3-rosinstall-generator \
#     python3-wstool \
#     python3-catkin-tools

# RUN apt-get install -y ros-noetic-cv-bridge libopencv-dev
# RUN apt-get install -y ros-noetic-tf

# WORKDIR /ladybug_sdk

# # Copy the deb package
# COPY ladybug-1.20.0.79_modified.deb /ladybug_sdk/

# # Copy my catkin package directory files
# COPY catkin_ws /home/ladybug_user/

# # Create flirimaging group and set up udev rules
# RUN groupadd -f flirimaging && \
#     mkdir -p /etc/udev/rules.d && \
#     echo 'ATTRS{idVendor}=="1e10", ATTRS{idProduct}=="3800", MODE="0664", GROUP="flirimaging"' > /etc/udev/rules.d/40-pgr-ladybug.rules && \
#     echo 'ATTRS{idVendor}=="1e10", ATTRS{idProduct}=="3801", MODE="0664", GROUP="flirimaging"' >> /etc/udev/rules.d/40-pgr-ladybug.rules && \
#     echo 'ATTRS{idVendor}=="1e10", ATTRS{idProduct}=="3802", MODE="0664", GROUP="flirimaging"' >> /etc/udev/rules.d/40-pgr-ladybug.rules && \
#     echo 'KERNEL=="raw1394", MODE="0664", GROUP="flirimaging"' >> /etc/udev/rules.d/40-pgr-ladybug.rules && \
#     echo 'KERNEL=="video1394*", MODE="0664", GROUP="flirimaging"' >> /etc/udev/rules.d/40-pgr-ladybug.rules && \
#     echo 'SUBSYSTEM=="firewire", GROUP="flirimaging"' >> /etc/udev/rules.d/40-pgr-ladybug.rules

# # Create non-root user first
# RUN useradd -m -d /home/ladybug_user -s /bin/bash ladybug_user && \
#     usermod -aG sudo ladybug_user && \
#     echo "ladybug_user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# # Add user to flirimaging group
# RUN usermod -aG flirimaging ladybug_user

# # Pre-accept EULA and install the package
# RUN echo "ladybug ladybug/accept-eula select true" | debconf-set-selections && \
#     echo "ladybug ladybug/accept-eula seen true" | debconf-set-selections && \
#     echo "ladybug ladybug/accept-eula boolean true" | debconf-set-selections && \
#     (dpkg -i ~/ladybug_sdk/ladybug-1.20.0.79_modified.deb || true) && \
#     apt-get update && \
#     apt-get -f install -y

# # Create necessary directories
# RUN mkdir -p /dev/bus/usb && \
#     mkdir -p /dev/raw1394

# # Switch to non-root user
# USER ladybug_user
# WORKDIR /home/ladybug_user

# CMD ["/bin/bash"]