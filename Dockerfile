# davetcoleman/rviz_visual_tools:kinetic
# Test setup for rviz_visual_tools

FROM osrf/ros:kinetic-desktop
MAINTAINER Dave Coleman dave@picknik.ai

ENV CATKIN_WS=/root/ws_catkin
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# download rviz_visual_tools source
RUN git clone https://github.com/davetcoleman/rviz_visual_tools.git -b ${ROS_DISTRO}-devel

# update apt-get because osrf image clears this cache and download deps
RUN apt-get -qq update && \
    apt-get -qq install -y \
        python-catkin-tools  \
        less \
        ssh \
        emacs \
        git-core \
        bash-completion \
        wget && \
    rosdep update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    rm -rf /var/lib/apt/lists/*

# HACK, replacing shell with bash for later docker build commands
RUN mv /bin/sh /bin/sh-old && \
    ln -s /bin/bash /bin/sh

# build repo
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN source /ros_entrypoint.sh && \
    catkin build --no-status
