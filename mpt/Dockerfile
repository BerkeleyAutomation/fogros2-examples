ARG UBUNTU_DISTRO=jammy
#rolling is alternative
ARG ROS_DISTRO=humble 
FROM ubuntu:${UBUNTU_DISTRO}

# Set up install, set tzdata
ARG UBUNTU_DISTRO
ARG ROS_DISTRO
ENV TZ=America/Vancouver
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Get ROS key
RUN apt update && apt install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install apt deps 
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${UBUNTU_DISTRO} main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y \
  ros-${ROS_DISTRO}-desktop \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  iproute2 \
  net-tools \
  python3-colcon-common-extensions \
  python3-pip \
  unzip \
  wireguard
RUN rm -rf /var/lib/apt/lists/*


RUN apt update && apt-get install -y git


# eigen3
# WORKDIR / 
# RUN git clone https://github.com/eigenteam/eigen-git-mirror.git
# RUN cd eigen-git-mirror && mkdir build && cd build && \
#     cmake .. &&\
#     make install

#Libccd 
WORKDIR /
RUN git clone https://github.com/UNC-Robotics/nigh.git && \
    git clone https://github.com/danfis/libccd.git 

WORKDIR /libccd
RUN mkdir build && cd build && \
    cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release .. && \
    make -j8 && make install

# fcl
WORKDIR /
RUN git clone https://github.com/flexible-collision-library/fcl.git && \
    cd fcl && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && make -j8 && make install  

# assimp
# reference: https://github.com/bferns/assimp-alpine-docker-build/blob/master/Dockerfile
RUN git clone https://github.com/assimp/assimp.git -b v5.0.0 
WORKDIR /assimp
RUN cmake CMakeLists.txt -G 'Unix Makefiles'; \
    make -j4; \
    make install; 
# for some reason, default installation is not successful
RUN cp /assimp/bin/* /usr/local/bin/ 

# Install python deps
# RUN pip install "git+https://github.com/skypilot-org/skypilot.git#egg=sky-callback&subdirectory=sky/callbacks/"
# Create FogROS2 worspace and build it

# RUN apt-get install -y libassimp-dev

ENV ROS_WS=/home/root/fog_ws
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}/src
COPY .  ${ROS_WS}/src/
RUN ls
WORKDIR ${ROS_WS}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
      colcon build --cmake-clean-cache

RUN cp -r /usr/local/lib/* ./install/mpt_ros/lib/

# setup entrypoint
ENV ROS_DISTRO=${ROS_DISTRO}
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]
CMD ["bash"]