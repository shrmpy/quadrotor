#!/usr/bin/env bash
set -e

# For reference wiki.ros.org/hector_quadrotor

if [ "$EUID" -ne 0 ]; then
    echo "This script uses functionality which requires root privileges"
    exit 1
fi

# base image 
acbuild --debug begin docker://osrf/ros:kinetic-desktop-full-xenial

# In the event of the script exiting, end the build
trap "{ export EXT=$?; acbuild --debug end && exit $EXT; }" EXIT

# Assign a VNC password with env variable
VNC_PASSWORD=secret

# Install dependencies
acbuild --debug run -- apt-get update
acbuild --debug run -- apt-get install -y \
                    dbus-x11 x11-utils x11vnc xvfb supervisor \
                    dwm suckless-tools stterm \
                    build-essential \
                    cmake \
                    imagemagick \
                    libboost-all-dev \
                    libgts-dev \
                    libjansson-dev \
                    libtinyxml-dev \
                    mercurial \
                    nodejs \
                    nodejs-legacy \
                    npm \
                    pkg-config \
                    psmisc 

acbuild run -- x11vnc -storepasswd $VNC_PASSWORD /etc/vncsecret
acbuild run -- chmod 444 /etc/vncsecret
acbuild port add vnc tcp 5900

# Make the container's entrypoint the supervisord
acbuild copy ./supervisord.conf /etc/supervisor/conf.d/supervisord.conf
acbuild set-exec -- /usr/bin/supervisord -c /etc/supervisor/conf.d/supervisord.conf
##acbuild set-exec -- /bin/sh -c "cd /root/gzweb; gserver --verbose; npm start;"

# Choose the user to run-as inside the container
acbuild --debug run -- adduser --system --home /home/gopher --shell /bin/bash --group --disabled-password gopher
acbuild --debug run -- usermod -a -G www-data gopher
##acbuild --debug set-user gopher

# clone gzweb
acbuild --debug run -- hg clone https://bitbucket.org/osrf/gzweb /root/gzweb
acbuild --debug run -- /bin/sh -c 'cd /root/gzweb; hg up default; . /opt/ros/kinetic/setup.sh; \
                    xvfb-run -s "-screen 0 1280x1024x24" ./deploy.sh -m local;'
acbuild port add web tcp 8080
acbuild port add gzq tcp 7681

# Download quadrotor
acbuild --debug run -- /bin/sh -c "mkdir -p /root/catkin_ws/src; . /opt/ros/kinetic/setup.sh; apt-get install -y \
                    ros-kinetic-teleop-twist-keyboard ros-kinetic-joystick-drivers ros-kinetic-geographic-msgs; \
                    rosinstall /root/catkin_ws/src /opt/ros/kinetic https://raw.githubusercontent.com/AS4SR/hector_quadrotor/kinetic-devel/tutorials.rosinstall; \
                    . /root/catkin_ws/src/setup.sh; \
                    rosdep install --from-path /root/catkin_ws/src --ignore-src --rosdistro kinetic --default-yes; \
                    cd /root/catkin_ws; catkin_make;" 

##acbuild --debug run -- /bin/sh -c ". /root/catkin_ws/devel/setup.sh; roslaunch hector_quadrotor_demo indoor_slam_gazebo.launch"

# Write the result
acbuild --debug set-name quad
acbuild --debug label add version 0.0.8
acbuild --debug write --overwrite quad-0.0.8-linux-amd64.aci
