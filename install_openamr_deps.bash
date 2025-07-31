#!/bin/bash

# This script installs the necessary dependencies for ROS2 and sets up a basic ROS2 workspace with Nav2 and linorobot2.
# This script was written by AÏM Arnold and TOBBAL Yannis, 16.06.2025

# Check if python3 is installed, install if missing
echo "Checking for python3"
if ! command -v python3 &> /dev/null; then
    echo "Python3 is not installed. Installing Python3..."
    sudo apt update -y && sudo apt install python3 python3-pip -y
else
    echo "Python3 found, checking for pip."
    if ! command -v python3 &> /dev/null; then
        echo "Pip not found, installing"
        sudo apt install python3-pip -y
    else
        echo "All good"
    fi
fi

# Check if ROS is installed by looking for ROS_DISTRO env variable
echo "Checking for ROS version"
if [ -z "$ROS_DISTRO" ]; then 
    echo "ROS is not installed. Preparing installation..."

    # Set up locales for ROS
    sudo apt update -y && sudo apt install locales -y
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # Add universe repo and install curl
    sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt update && sudo apt install curl -y

    # Download and install the latest ROS apt source for your Ubuntu version
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
    sudo apt install /tmp/ros2-apt-source.deb

    # Install ROS dev tools and update system
    sudo apt update -y && sudo apt install ros-dev-tools -y
    sudo apt update -y
    sudo apt upgrade -y

    # Detect Ubuntu version and install the corresponding ROS distribution
    UbuntuVersion=$(lsb_release -rs)
    if [[ "$UbuntuVersion" == "20.04" ]]; then
        echo "Installing ROS Foxy"
        sudo apt install -y ros-foxy-desktop
        export ROS_DISTRO=foxy
    elif [[ "$UbuntuVersion" == "22.04" ]]; then
        echo "Installing ROS Humble"
        sudo apt install -y ros-humble-desktop
        export ROS_DISTRO=humble
    elif [[ "$UbuntuVersion" == "24.04" ]]; then
        echo "Installing ROS Jazzy"
        sudo apt install -y ros-jazzy-desktop
        export ROS_DISTRO=jazzy
    else
        echo "Unsupported Ubuntu version: $UbuntuVersion. Please install ROS manually."
        exit 1
    fi

    # Source ROS setup and add to .bashrc
    source /opt/ros/$ROS_DISTRO/setup.bash
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $HOME/.bashrc
    echo "export ROS_DOMAIN_ID=47" >> $HOME/.bashrc

    # Install rosdep and initialize
    sudo apt update -y
    sudo apt install python3-rosdep -y
    sudo rosdep init
    rosdep update
fi

# Print ROS version
echo "ROS version: $ROS_DISTRO"

# Check for Colcon build tool, install if missing
echo "Setting up Colcon"
if ! command -v colcon &> /dev/null ; then
    echo "Colcon is not installed. Installing Colcon..."
    sudo apt install python3-colcon-common-extensions -y
else
    echo "Colcon found."
fi

# Check for rqt GUI tool, install if missing
echo "Checking for rqt"
if ! command -v rqt &> /dev/null ; then
    echo "rqt is not installed. Installing rqt..."
    sudo apt install ros-$ROS_DISTRO-rqt* -y
else
    echo "rqt found."
fi

# Check for ROS workspace, create if missing
echo "Checking for Ros Workspace"
if [ ! -d $HOME/ros2_ws ]; then
    echo "Creating ROS workspace at $HOME/ros2_ws"
    mkdir -p $HOME/ros2_ws/src
    cd $HOME/ros2_ws
    colcon build --symlink-install
    echo "source $HOME/ros2_ws/install/setup.bash" >> $HOME/.bashrc
else
    echo "ROS workspace already exists at $HOME/ros2_ws"
fi

# Check for Nav2 package, install if missing
echo "Checking for Nav2"
if ! ros2 pkg list | grep "nav2_bringup"; then
    echo "Nav2 is not installed. Installing Nav2 ..."
    sudo apt install ros-$ROS_DISTRO-navigation2 -y
    sudo apt install ros-$ROS_DISTRO-nav2-bringup -y
else
    echo "Nav2 found."
fi

# Check for web video server, install if missing
echo "Checking for web video server"
if ! ros2 pkg list | grep "web_video_server"; then
    echo "Web video server is not installed. Installing ..."
    sudo apt install ros-$ROS_DISTRO-web-video-server -y
else
    echo "Web video server found."
fi

# Check for rosbridge server, install if missing
echo "Checking for rosbridge server"
if ! ros2 pkg list | grep "rosbridge_server"; then
    echo "Rosbridge server is not installed. Installing ..."
    sudo apt install ros-$ROS_DISTRO-rosbridge-server -y
else
    echo "Rosbridge server found."
fi

# Check for linorobot2, install if missing
echo "Checking for linorobot2"
if ! ros2 pkg list | grep "linorobot2_bringup"; then
    # Install LIDAR ROS2 drivers
    echo "Linorobot2 is not installed. Installing ..."
    cd $HOME/ros2_ws/src
    git clone https://github.com/Slamtec/sllidar_ros2.git
    colcon build
    source  $HOME/ros2_ws/install/setup.bash
    sudo cp sllidar_ros2/scripts/rplidar.rules /etc/udev/rules.d

    # Download and install micro-ROS
    cd $HOME/ros2_ws/src
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup src/micro_ros_setup
    sudo apt install python3-vcstool build-essential
    sudo apt update && rosdep update
    rosdep install --from-path src --ignore-src -y
    colcon build
    source install/setup.bash

    # Setup micro-ROS agent
    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
    source install/setup.bash

    # Download linorobot2 source code
    cd $HOME/ros2_ws
    git clone -b $ROS_DISTRO https://github.com/linorobot/linorobot2 src/linorobot2

    # Ignore Gazebo simulation package (optional)
    cd src/linorobot2/linorobot2_gazebo
    touch COLCON_IGNORE

    # Install linorobot2 dependencies and build
    cd $HOME/ros2_ws
    rosdep update && rosdep install --from-path src --ignore-src -y --skip-keys microxrcedds_agent
    colcon build
    source install/setup.bash
else
    echo "Linorobot2 found."
fi

pip3 install flask

# Set up environment variables for linorobot2
echo "export LINOROBOT2_BASE=2wd" >> ~/.bashrc
echo "export LINOROBOT2_LASER_SENSOR=rplidar" >> ~/.bashrc
source ~/.bashrc

# Print completion messages and usage instructions
echo "Linorobot2 setup complete."
echo "Installation complete. Please restart your terminal or run 'source ~/.bashrc' to apply the changes."
echo "To start the micro-ROS agent, run 'ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888' in a new terminal."
echo "To start the Nav2 stack, run 'ros2 launch linorobot2_bringup nav2_bringup_launch.py' in a new terminal."
echo "To start the web video server, run 'ros2 launch linorobot2_bringup web_video_server_launch.py' in a new terminal."
echo "To start the rqt GUI, run 'rqt' in a new terminal."
