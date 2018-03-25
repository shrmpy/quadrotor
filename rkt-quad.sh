#!/usr/bin/env bash
set -e

# For reference https://github.com/AS4SR/hector_quadrotor

if [ "$EUID" -ne 0 ]; then
    echo "This script uses functionality which requires root privileges"
    exit 1
fi

# base image 
acbuild begin docker://osrf/ros:kinetic-desktop-full-xenial

# In the event of the script exiting, end the build
trap "{ export EXT=$?; acbuild --debug end && exit $EXT; }" EXIT

# Assign a VNC password with env variable
VNC_PASSWORD=secret

# Install dependencies
acbuild run -- apt-get update
acbuild run -- apt-get install -y \
                                  dbus-x11 x11-utils x11vnc xvfb supervisor \
                                  dwm suckless-tools stterm 

acbuild run -- x11vnc -storepasswd $VNC_PASSWORD /etc/vncsecret
acbuild run -- chmod 444 /etc/vncsecret
acbuild port add vnc tcp 5900

# Make the container's entrypoint the supervisord
acbuild copy ./supervisord.conf /etc/supervisor/conf.d/supervisord.conf
acbuild set-exec -- /usr/bin/supervisord -c /etc/supervisor/conf.d/supervisord.conf

# Choose the user to run-as inside the container
acbuild --debug run -- adduser --system --home /home/gopher --shell /bin/bash --group --disabled-password gopher
acbuild --debug run -- usermod -a -G www-data gopher
##acbuild --debug set-user gopher

# Download gazebo
##acbuild --debug run -- /bin/sh -c "mkdir -p /root/catkin_ws/src; . /opt/ros/kinetic/setup.sh; apt-get install -y \
                                     ros-kinetic-teleop-twist-keyboard ros-kinetic-joystick-drivers python-wstool ros-kinetic-geographic-msgs; \
                                     cd /root/catkin_ws; rosdep install --from-path src --ignore-src --rosdistro kinetic; \
                                     wstool init src https://raw.githubusercontent.com/AS4SR/hector_quadrotor/kinetic-devel/tutorials.rosinstall; \
                                     catkin_make; . /root/catkin_ws/devel/setup.sh;"
##acbuild --debug run -- /bin/sh -c "roslaunch hector_quadrotor_demo indoor_slam_gazebo.launch"


# Write the result
acbuild --debug set-name quad
acbuild --debug label add version 0.0.1
acbuild --debug write --overwrite quad-0.0.1-linux-amd64.aci
