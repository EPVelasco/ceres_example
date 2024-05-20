FROM osrf/ros:noetic-desktop-focal

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y apt-utils curl wget git bash-completion build-essential sudo && rm -rf /var/lib/apt/lists/*

# Now create the user
ARG UID=1000
ARG GID=1000
RUN addgroup --gid ${GID} ceres
RUN adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} ceres
RUN usermod -a -G dialout ceres
RUN mkdir config && echo "ros ALL=(ALL) NOPASSWD: ALL" > config/99_aptget
RUN cp config/99_aptget /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

RUN apt-get update && apt-get install -y \
    git build-essential ros-noetic-perception-pcl ros-noetic-rviz ros-noetic-eigen-conversions iputils-ping screen vim nano && \
    rm -rf /var/lib/apt/lists/*

ENV HOME /home/ceres
RUN mkdir -p ${HOME}/catkin_ws/library

# Ceres dependeces
RUN apt-get update && apt-get install cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev -y

RUN cd ${HOME}/catkin_ws/library && git clone https://ceres-solver.googlesource.com/ceres-solver && cd ceres-solver && mkdir ceres-bin && cd ceres-bin 
RUN cd ${HOME}/catkin_ws/library/ceres-solver/ceres-bin/ && cmake .. && make && make install

# Lidar Odometry dependeces
RUN apt-get update && \
    apt-get install -y --no-install-recommends \       
    ros-noetic-hector-trajectory-server     
   
# extras tools
RUN apt-get update && apt-get -y install vim terminator ranger

RUN . /opt/ros/noetic/setup.sh && \
    mkdir -p ${HOME}/catkin_ws/src && \ 
    cd ${HOME}/catkin_ws/src   
    




# copy odom_estimation folder to docker
RUN mkdir ${HOME}/catkin_ws/src/odom_estimation_pc/
COPY ./odom_estimation_pc  ${HOME}/catkin_ws/src/odom_estimation_pc/

#RUN . /opt/ros/noetic/setup.sh && \ 
#    cd ${HOME}/catkin_ws/ && catkin_make 


# add ceres_example repositoy
#RUN cd ${HOME}/catkin_ws/src/  && git clone https://github.com/EPVelasco/ceres_example.git
RUN mkdir ${HOME}/catkin_ws/src/ceres_example/
COPY ./ceres_example ${HOME}/catkin_ws/src/ceres_example/

#ruta de trabajo
WORKDIR /home/ceres/catkin_ws 
