FROM nvidia/cuda:11.6.1-devel-ubuntu20.04

ARG rosdistro=galactic
ARG cudnn_version=8.2.4.15-1+cuda11.4
ARG tensorrt_version=8.2.4-1+cuda11.4
ARG clang_format_version=14.0.6
ARG UID=1000
ARG USER=developer
RUN useradd -m -u ${UID} ${USER}
ENV DEBIAN_FRONTEND=noninteractive \
    HOME=/home/${USER}
WORKDIR ${HOME}

RUN apt-get update && apt-get install -y \
    curl wget git build-essential sudo cmake \
    software-properties-common gnupg lsb-release apt-transport-https \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Install Golang (Add Go PPA for shfmt)
RUN add-apt-repository ppa:longsleep/golang-backports
# pacmod
RUN echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list
# ros
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update && apt-get install -y \
    # ros
    ros-${rosdistro}-desktop \
    python3-colcon-common-extensions python3-flake8 python3-pip python3-pytest-cov \
    python3-rosdep python3-setuptools python3-vcstool \
    # rmw_implementation
    ros-${rosdistro}-rmw-cyclonedds-cpp \
    # pacmod
    ros-${rosdistro}-pacmod3 \
    # Install Autoware Universe dependencies
    geographiclib-tools \
    # Install Golang (Add Go PPA for shfmt)
    golang \
    # cudnn
    libcudnn8=${cudnn_version} libcudnn8-dev=${cudnn_version} \
    # tensorrt
    libnvinfer8=${tensorrt_version} libnvonnxparsers8=${tensorrt_version} \
    libnvparsers8=${tensorrt_version} libnvinfer-plugin8=${tensorrt_version} \
    libnvinfer-dev=${tensorrt_version} libnvonnxparsers-dev=${tensorrt_version} \
    libnvparsers-dev=${tensorrt_version} libnvinfer-plugin-dev=${tensorrt_version} \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# cudnn
RUN apt-mark hold libcudnn8 libcudnn8-dev
# tensorrt
RUN apt-mark hold libnvinfer8 libnvonnxparsers8 libnvparsers8 libnvinfer-plugin8 libnvinfer-dev \
    libnvonnxparsers-dev libnvparsers-dev libnvinfer-plugin-dev

RUN python3 -m pip install --upgrade pip \
    && python3 -m pip install \
    # ros
    flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated \
    flake8-docstrings flake8-import-order flake8-quotes \
    pytest-repeat pytest-rerunfailures pytest \
    setuptools \
    # Install Autoware Core dependencies
    gdown \
    # pre commit
    pre-commit clang-format==${clang_format_version}

# Install Autoware Universe dependencies
RUN geographiclib-get-geoids egm2008-1

COPY --chown=${USER} ./autoware ${HOME}/autoware
SHELL ["/bin/bash", "-c"]
RUN apt-get update \
    && cd autoware \
    && echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
    && source /opt/ros/${rosdistro}/setup.bash \
    && rosdep init \
    && rosdep update \
    && rosdep install -y --from-paths src --ignore-src --rosdistro ${rosdistro} \
    && MAKEFLAGS="-j 2" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN chown -R ${USER} .ros
USER ${USER}
RUN echo "source ~/autoware/install/setup.bash" >> .bashrc
CMD ["/bin/bash"]