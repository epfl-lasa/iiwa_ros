# Handle ros distro
ARG ROS_DISTRO=noetic
FROM ghcr.io/aica-technology/ros-ws:${ROS_DISTRO}

# User provided arguments
ARG HOST_GID=1000
ARG GIT_NAME=""
ARG GIT_EMAIL=""
ARG USE_SIMD=OFF

# Tell docker we want to use bash instead of sh in general
SHELL ["/bin/bash", "-c"]

### Add the user to the current GID of the host to avoid permisson issues in volumes
# AICA uses the same name for user and user group
ENV USER_GROUP=${USER}
USER root
RUN if [ "HOST_GID" != "1000"] ; \
    then groupadd --gid ${HOST_GID} host_group && \
    usermod ${USER} -g ${HOST_GID} && \ 
    usermod ${USER} -a -G ${USER_GROUP}; fi
USER ${USER}

# Setup git identity
RUN git config --global user.name "${GIT_NAME}"
RUN git config --global user.email "${GIT_EMAIL}"

# Setup python version for noetic
RUN sudo apt update
RUN if [ "${ROS_DISTRO}" == "noetic" ] ; \
    then sudo apt install python-is-python3 ; fi


### Add a few tools
RUN sudo apt-get update && sudo apt-get install -y \
    bash-completion \
    silversearcher-ag \
    apt-transport-https \
    less \
    alsa-utils \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers \
    net-tools \
    netbase \
    && sudo apt-get upgrade -y && sudo apt-get clean

# Install gazebo (9 or 11 depending on distro)
WORKDIR /home/${USER}
RUN sudo apt-get update
RUN if [ "$ROS_DISTRO" = "noetic" ] ; then sudo apt-get install -y gazebo11 ; fi
RUN if [ "$ROS_DISTRO" = "melodic" ] ; then sudo apt-get install -y gazebo9 ; fi

# Install gaezbo ros packages
RUN sudo apt install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-gazebo-ros-control

# Handle SIMD option
RUN if [ "${USE_SIMD}" = "ON" ] ; \ 
    then export CMAKE_CXX_FLAGS="-march=native -faligned-new" ; fi

### Install all dependencies of IIWA ROS
# Clone KUKA FRI (need to be root to clone private repo)
WORKDIR /tmp
USER root
RUN --mount=type=ssh git clone git@github.com:epfl-lasa/kuka_fri.git
WORKDIR /tmp/kuka_fri
RUN if [ "${USE_SMID}" != "ON" ] ; \
    then wget https://gist.githubusercontent.com/matthias-mayr/0f947982474c1865aab825bd084e7a92/raw/244f1193bd30051ae625c8f29ed241855a59ee38/0001-Config-Disables-SIMD-march-native-by-default.patch \
    ; fi
RUN --mount=type=ssh  if [ "${USE_SMID}" != "ON" ] ; \
    then git am 0001-Config-Disables-SIMD-march-native-by-default.patch ; fi

# Transfer repo back to original user after root clone
WORKDIR /tmp
RUN chown -R ${USER}:${HOST_GID} kuka_fri

# Install kuka Fri as USER
USER ${USER}
RUN cd kuka_fri && ./waf configure && ./waf && sudo ./waf install

# Install SpaceVecAlg
RUN git clone --recursive https://github.com/jrl-umi3218/SpaceVecAlg.git
RUN cd SpaceVecAlg && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=OFF .. \
    && make -j && sudo make install

# Install RBDyn
RUN git clone --recursive https://github.com/jrl-umi3218/RBDyn.git
RUN cd RBDyn && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=OFF .. \
    && make -j && sudo make install

# Install mc_rbdyn_urdf
RUN git clone --recursive -b v1.1.0 https://github.com/jrl-umi3218/mc_rbdyn_urdf.git
RUN cd mc_rbdyn_urdf && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_BINDING=OFF .. \
    && make -j && sudo make install

# Install corrade
RUN git clone https://github.com/mosra/corrade.git
RUN cd corrade && git checkout 0d149ee9f26a6e35c30b1b44f281b272397842f5 \
    && mkdir build && cd build && cmake .. && make -j && sudo make install

# Install robot_controller
RUN git clone https://github.com/epfl-lasa/robot_controllers.git
RUN cd robot_controllers && mkdir build && cd build \
    && cmake .. && make -j && sudo make install

# Remove temporari files
RUN sudo ldconfig
RUN rm -rf /tmp/*

# ### Install IIWA ROS
# WORKDIR /home/${USER}/ros_ws/src
# RUN git clone -b featuresim https://github.com/epfl-lasa/iiwa_ros.git

# Copy IIWA ROS folder inside docker
WORKDIR /home/${USER}/ros_ws
COPY --chown=${USER} ./ ./src/iiwa_ros

### Add environement variables to bashrc
WORKDIR /home/${USER}

# Give bashrc back to user
RUN sudo chown -R ${USER}:${HOST_GID} .bashrc

# Add cmake option to bash rc if needed
RUN if [ "${USE_SIMD}" = "ON" ] ; \
    then echo "export ENABLE_SIMD=ON" >> /home/${USER}/.bashrc ; fi

### Build ros workspace
WORKDIR /home/${USER}/ros_ws
RUN source /home/${USER}/.bashrc && rosdep install --from-paths src --ignore-src -r -y
RUN source /home/${USER}/.bashrc && catkin_make;

### Final apt clean
RUN sudo apt update && sudo apt upgrade -y && sudo apt clean -y
