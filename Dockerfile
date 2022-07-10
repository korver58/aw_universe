FROM nvidia/cuda:11.6.1-cudnn8-devel-ubuntu20.04

ARG UID=1000
ARG USER=developer
RUN useradd -m -u ${UID} $USER
ENV DEBIAN_FRONTEND=noninteractive \
    HOME=/home/$USER/
WORKDIR /home/$USER/

RUN apt-get update && apt-get install -y \
    curl wget git build-essential sudo \
    software-properties-common gnupg2 lsb-release \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# install ros galactic
ARG ROS_DISTRO=galactic
ARG INSTALL_PACKAGE=desktop

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-${INSTALL_PACKAGE} \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install some pip packages needed for testing
RUN python3 -m pip install -U \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  setuptools

# Initialize rosdep
RUN rosdep init \
    && rosdep update


# rmw_implementation
ARG rosdistro=galactic
# ARG rmw_implementation=rmw_cyclonedds_cpp
# ARG rmw_implementation_dashed=$(eval sed -e "s/_/-/g" <<< "${rmw_implementation}")
RUN apt-get update && apt-get install -y \
    ros-${rosdistro}-rmw-cyclonedds-cpp \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# ARG rmw_implementation_dashed=$(eval sed -e "s/_/-/g" <<< "rmw_cyclonedds_cpp")


# # (Optional) You set the default RMW implementation in the ~/.bashrc file.
# echo '' >> ~/.bashrc && echo "export RMW_IMPLEMENTATION=${rmw_implementation}" >> ~/.bashrc

# pacmod
# Taken from https://github.com/astuff/pacmod3#installation
RUN sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'

RUN apt-get update && apt-get install -y \
    apt-transport-https ros-${rosdistro}-pacmod3 \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


RUN python3 -m pip install -U \
    gdown

RUN apt-get update && apt-get install -y \
    geographiclib-tools \
    && apt-get clean && rm -rf /var/lib/apt/lists/*
RUN geographiclib-get-geoids egm2008-1

# pre commit
ARG clang_format_version=14.0.6
RUN python3 -m pip install pre-commit clang-format==${clang_format_version}

# Install Golang (Add Go PPA for shfmt)
RUN add-apt-repository ppa:longsleep/golang-backports
RUN apt-get update && apt-get install -y \
    golang \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

ARG tensorrt_version="8.2.4-1+cuda11.4"
RUN apt-get update && apt-get install -y \
    libnvinfer8=${tensorrt_version} libnvonnxparsers8=${tensorrt_version} \
    libnvparsers8=${tensorrt_version} libnvinfer-plugin8=${tensorrt_version} \
    libnvinfer-dev=${tensorrt_version} libnvonnxparsers-dev=${tensorrt_version} \
    libnvparsers-dev=${tensorrt_version} libnvinfer-plugin-dev=${tensorrt_version} \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-mark hold libnvinfer8 libnvonnxparsers8 libnvparsers8 libnvinfer-plugin8 libnvinfer-dev \
    libnvonnxparsers-dev libnvparsers-dev libnvinfer-plugin-dev

RUN add-apt-repository universe

# RUN apt-get update && apt-get install -y \
#     ros-galactic-diagnostic-updater \
#     ros-galactic-osrf-testing-tools-cpp \
#     ros-galactic-filters \
#     python3-flask \
#     ros-galactic-nav2-costmap-2d \
#     ros-galactic-ros-testing \
#     && apt-get clean && rm -rf /var/lib/apt/lists/*


COPY --chown=${USER} . /home/$USER/autoware
SHELL ["/bin/bash", "-c"]
RUN apt-get update \
    && cd autoware \
    && mkdir src \
    && vcs import src < autoware.repos \
    && source /opt/ros/galactic/setup.bash \
    && rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

USER $USER
CMD ["/bin/bash"]