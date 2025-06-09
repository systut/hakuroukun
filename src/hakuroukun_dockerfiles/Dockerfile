FROM osrf/ros:noetic-desktop-full 

# copy install scripts
COPY ./hakuroukun_dockerfiles/install /root/install

# copy directory specific to scart
COPY ./ /root/catkin_ws/src/

# update the new ROS repository key
RUN rm /etc/apt/sources.list.d/ros1-latest.list \
    && rm /usr/share/keyrings/ros1-latest-archive-keyring.gpg

RUN apt-get update -y \
    && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros-apt-source.deb \
    && rm -f /tmp/ros-apt-source.deb

RUN apt-get upgrade -y 

# update and install dependencies
RUN /bin/sh -e -c /root/install/i_dep.sh

# set environment variable for ros
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> /root/.bashrc

# install ros dependencies
WORKDIR /root/catkin_ws 
RUN catkin config --extend /opt/ros/noetic
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build ROS packages
# RUN catkin build && \
#     chmod 777 /root/catkin_ws/devel/setup.bash && \
#     chmod 755 /root

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics