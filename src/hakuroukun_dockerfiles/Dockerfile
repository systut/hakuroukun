FROM osrf/ros:noetic-desktop-full 

# copy install scripts
COPY ./hakuroukun_dockerfiles/install /root/install

# copy directory specific to scart
COPY ./ /root/catkin_ws/src/

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
#    chmod 777 /root/catkin_ws/devel/setup.bash && \
#    chmod 755 /root

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics