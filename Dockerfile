# picknik/rviz_visual_tools:eloquent
# Test setup for rviz_visual_tools

FROM ros:eloquent-ros-base-bionic
MAINTAINER Mike Lautman mike@picknik.ai

ENV TERM xterm

ENV GIT_BRANCH=${ROS_DISTRO}-devel

# Setup catkin workspace
ENV ROS_WS=/opt/ws_moveit
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

# update apt-get because osrf image clears this cache and download deps
RUN apt-get -qq update && \
    apt-get -qq dist-upgrade && \
    apt-get -qq install -y wget \
        curl sudo xvfb mesa-utils ccache ssh curl gnupg2 lsb-release \
        clang clang-format-3.9 clang-tidy clang-tools \
        # Some python dependencies for working with ROS2
        python3-colcon-common-extensions \
        python3-pip \
        python-rosdep \
        python3-wstool \
        python3-vcstool \
        python3-rospkg-modules \
        python3-rosdistro-modules && \
    rosdep update -q && \
    cd $ROS_WS/src && \
    git clone https://github.com/picknikrobotics/rviz_visual_tools.git -b ${GIT_BRANCH} && \
    # Remove folders declared as COLCON_IGNORE
    find -L . -name COLCON_IGNORE -printf "%h\0" | xargs -0 rm -rf && \
    rosdep install -q -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} --as-root=apt:false && \
    # Clear apt-cache to reduce image size
    rm -rf /var/lib/apt/lists/* && \
    . /opt/ros/${ROS_DISTRO}/setup.sh &&\
    cd $ROS_WS/ && \
    colcon build \
            --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
              --ament-cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
            --event-handlers desktop_notification- status-
