FROM dustynv/pytorch:2.1-r35.3.1

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Install all needed deps and compile the mesa llvmpipe driver from source.
RUN sudo apt-get update \
    && apt-get install -y software-properties-common \
    && apt-get update \
    && add-apt-repository ppa:kisak/kisak-mesa \
    && apt-get update \
    && apt-get -y upgrade \
    && apt-get update && apt-get install -y wget \
    && wget http://packages.ros.org/ros.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y python3-catkin-tools python3-pip

RUN apt-get install -y ros-noetic-rosserial-arduino \
    ros-noetic-pid \
    ros-noetic-joy \
    ros-noetic-joy-teleop \
    ros-noetic-sbg-driver \
    ros-noetic-cv-bridge \
    ros-noetic-image-view \
    ros-noetic-rqt-gui \
    ros-noetic-smach-ros

COPY requirements.txt requirements.txt

RUN pip install --no-cache-dir -r requirements.txt \
    && pip install --no-deps --no-cache-dir ultralytics==8.0.207 \
    && rm -f /usr/local/bin/cmake && rm -rf /root/.cache/pip && rm requirements.txt 

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
            libjpeg-dev \
		  zlib1g-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

RUN git clone --branch release/0.16 --recursive --depth=1 https://github.com/pytorch/vision torchvision && \
    cd torchvision && \
    git checkout release/0.16 && \
    python3 setup.py bdist_wheel && \
    cp dist/torchvision*.whl /opt && \
    pip3 install --no-cache-dir --verbose /opt/torchvision*.whl && \
    cd ../ && \
    rm -rf torchvision \
    && echo "source /AUV-2024/catkin_ws/devel/setup.bash" >> ~/.bashrc 

ARG UBUNTU_RELEASE_YEAR=20
ARG CUDA_MAJOR=11
ARG CUDA_MINOR=8.0
ARG ZED_SDK_MAJOR=4
ARG ZED_SDK_MINOR=0
    
# Setup the ZED SDK
RUN apt-get update -y || true ; apt-get install --no-install-recommends lsb-release wget less udev sudo zstd build-essential cmake python3 python3-pip libpng-dev libgomp1 -y ; \
    #python3 -m pip install --upgrade pip ; \
    python3 -m pip install numpy opencv-python ; \
    wget -q -O ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run https://download.stereolabs.com/zedsdk/${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}/cu${CUDA_MAJOR}${CUDA_MINOR%.*}/ubuntu${UBUNTU_RELEASE_YEAR} && \
    chmod +x ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run ; ./ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run -- silent skip_tools skip_cuda && \
    ln -sf /lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/lib/x86_64-linux-gnu/libusb-1.0.so && \
    rm ZED_SDK_Linux_Ubuntu${UBUNTU_RELEASE_YEAR}.run && \
    rm -rf /var/lib/apt/lists/*