ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Install essential build tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    libeigen3-dev \
    libboost-all-dev \
    libtbb-dev \
    coinor-libipopt-dev \
    libconsole-bridge-dev \
    liboctomap-dev \
    liburdfdom-dev \
    libassimp-dev \
    libpcl-dev \
    sudo \
    # Qt dependencies needed for Qt-Advanced-Docking-System
    qtbase5-dev \
    qtbase5-private-dev \
    libqt5x11extras5-dev \
    # Pre-install some dependencies to avoid building them
    libompl-dev \
    libfcl-dev \
    && rm -rf /var/lib/apt/lists/*

# Install additional dependencies for Tesseract
RUN apt-get update && apt-get install -y \
    libgtest-dev \
    libgmock-dev \
    libomp-dev \
    libccd-dev \
    libnlopt-dev \
    liboctomap-dev \
    && rm -rf /var/lib/apt/lists/*

# gazebo
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    apt update && \
    apt install -y libgz-rendering7-dev libgz-common5-dev libgz-common5-profiler-dev

# Create a non-root user
ARG USERNAME=gmruser
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Switch to the new user
USER ${USERNAME}:${USERNAME}

# QTAdvancedDocking
WORKDIR /home/${USERNAME}/deps
RUN git clone https://github.com/githubuser0xFFFF/Qt-Advanced-Docking-System.git && \
    cd Qt-Advanced-Docking-System && \
    mkdir build && cd build && \
    cmake .. && \
    make && \
    sudo make install

# Create workspace for Tesseract
RUN mkdir -p /home/${USERNAME}/deps/tesseract_ws/src
WORKDIR /home/${USERNAME}/deps/tesseract_ws/src

RUN git clone --branch 0.29.1 --depth 1 https://github.com/tesseract-robotics/tesseract && \
    git clone --branch 0.29.1 --depth 1 https://github.com/tesseract-robotics/tesseract_planning && \
    git clone --branch 0.29.1 --depth 1 https://github.com/tesseract-robotics/trajopt.git && \
    git clone --branch 0.29.2 --depth 1 https://github.com/tesseract-robotics/tesseract_ros2.git

# Import dependencies one by one with --skip-existing flag
RUN vcs import --skip-existing < trajopt/dependencies.repos
RUN vcs import --skip-existing < tesseract/dependencies.repos
RUN vcs import --skip-existing < tesseract/dependencies_with_ext.repos
RUN vcs import --skip-existing < tesseract_planning/dependencies.repos
RUN vcs import --skip-existing < tesseract_ros2/dependencies.repos

# Configure tesseract_qt remote
WORKDIR /home/$USERNAME/deps/tesseract_ws/src/tesseract_qt
RUN git remote rename origin upstream
RUN git remote add origin https://github.com/GrayMatter-Robotics/tesseract_qt.git && \
    git fetch origin && \
    git checkout fix-build-issues

# Install ROS dependencies (need to switch to root temporarily)
USER root
RUN apt-get update && \
    rosdep update && \
    cd /home/${USERNAME}/deps/tesseract_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/* && \
    chown -R ${USERNAME}:${USERNAME} /home/${USERNAME}/deps/tesseract_ws

# Switch back to the user
USER ${USERNAME}:${USERNAME}

# Build the workspace - use bash explicitly for source command
WORKDIR /home/${USERNAME}/deps/tesseract_ws
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --cmake-args -DCMAKE_CXX_COMPILER="/usr/bin/g++" \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_PREFIX_PATH="/usr/local/lib/cmake/qtadvanceddocking-qt5"

# Set up environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source /home/${USERNAME}/deps/tesseract_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc

# Set ROS logs to pretty print
ENV RCUTILS_CONSOLE_OUTPUT_FORMAT="{time}[{severity}] [{name}] ({function_name}() at {file_name}:{line_number}); {message}"
ENV RCUTILS_COLORIZED_OUTPUT="1"

WORKDIR /home/${USERNAME}/deps/tesseract_ws

CMD ["/bin/bash"]