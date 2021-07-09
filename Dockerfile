ARG BASE_IMAGE=osrf/ros
ARG BASE_TAG=melodic-desktop-full

FROM ${BASE_IMAGE}:${BASE_TAG} AS project-dependencies

ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM 1

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

RUN apt-get update && apt-get install -y \
	sudo wget curl git nano bash-completion \
	build-essential \
	openssh-server \
	libboost-python-dev \
	mesa-utils \
	apt-transport-https \
    ca-certificates \
    python-pip \
    && apt-get upgrade -y \
	&& apt-get clean \
	&& rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update && apt-get install -y \
    ros-melodic-ros-control \
    ros-melodic-ros-controllers \
    ros-melodic-joint-state-publisher-gui \
	libignition-math2-dev \
	gazebo9 \
	libgazebo9-dev \
	&& apt-get upgrade -y \
	&& apt-get clean \
	&& rm -rf /var/lib/apt/lists/*

RUN mkdir /root/.ssh/
RUN touch /root/.ssh/known_hosts
RUN ssh-keyscan github.com >> /root/.ssh/known_hosts

# install external dependencies
WORKDIR /tmp

# install SpaceVecAlg
RUN git clone --recursive https://github.com/costashatz/SpaceVecAlg.git
RUN cd SpaceVecAlg && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF .. \
    && make -j && sudo make install

# install RBDyn
RUN git clone --recursive https://github.com/costashatz/RBDyn.git
RUN cd RBDyn && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF .. \
    && make -j && sudo make install

# install mc_rbdyn_urdf
RUN git clone --recursive https://github.com/costashatz/mc_rbdyn_urdf.git
RUN cd mc_rbdyn_urdf && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_SIMD=ON -DPYTHON_BINDING=OFF .. \
    && make -j && sudo make install

# install corrade
RUN git clone https://github.com/mosra/corrade.git
RUN cd corrade && git checkout 0d149ee9f26a6e35c30b1b44f281b272397842f5 \
    && mkdir build && cd build && cmake .. \
    && make -j && sudo make install

# install robot_controllers
RUN git clone https://github.com/epfl-lasa/robot_controllers.git
RUN cd robot_controllers && mkdir build && cd build \
    && cmake .. && make -j && sudo make install

# install kuka_fri
RUN --mount=type=ssh git clone git@github.com:epfl-lasa/kuka_fri.git
RUN cd kuka_fri && ./waf configure && ./waf && sudo ./waf install

# install control libraries
RUN git clone --single-branch --branch develop https://github.com/epfl-lasa/control_libraries
RUN bash control_libraries/source/install.sh --auto

RUN sudo ldconfig
RUN rm -rf /tmp/*

FROM project-dependencies as ros-ws

# Now create the same user as the host itself
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} ros
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} ros
RUN usermod -a -G dialout ros
RUN echo "ros ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

# Choose to run as user
USER ros

# Change HOME environment variable
ENV HOME /home/ros

# workspace setup
RUN mkdir -p ~/ros_ws/src

RUN cd ~/ros_ws/src && /bin/bash -c "source /ros_entrypoint.sh; catkin_init_workspace"
RUN cd ${HOME}/ros_ws && /bin/bash -c "source /ros_entrypoint.sh; catkin_make"
WORKDIR ${HOME}/ros_ws/src

# Change .bashrc
COPY docker/update_bashrc /sbin/update_bashrc
RUN sudo chmod +x /sbin/update_bashrc ; sudo chown ros /sbin/update_bashrc ; sync ; /bin/bash -c /sbin/update_bashrc ; sudo rm /sbin/update_bashrc

# ros user with everything pre-built
FROM ros-ws AS ros-user

COPY --chown=ros . .
RUN rm -rf ./docker ./Dockerfile
RUN cd ${HOME}/ros_ws && /bin/bash -c "source /ros_entrypoint.sh; catkin_make"

# Change entrypoint to source ~/.bashrc and start in ~
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown ros /ros_entrypoint.sh ;

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]


# dev user to be used with shared volume
FROM ros-ws AS dev-user

# Change entrypoint to source ~/.bashrc and start in ~
COPY docker/ros_entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown ros /ros_entrypoint.sh ;

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
