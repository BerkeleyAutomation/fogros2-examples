# mpt_ros

ROS wrapper on Motion Planning Templates

# Installation

This should be checked out under the `src` directory of an existing catkin workspace.  It depends on two submodules that need to be cloned too.  If `catkin_ws` is the name of the workspace, then the following command will check out this repo.

    % cd ~/catkin_ws/src
    % git clone https://github.com/jeffi/mpt_ros.git
    % cd mpt_ros
    % git submodule init
    % git submodule update
    
The following libraries also need to be installed on the system to compile:

    % sudo apt install libeigen3-dev libassimp-dev libccd-dev
    
And FCL should be cloned, compiled, and installed separately (the version in the apt repo is incompatible).

    % cd ~
    % git clone https://github.com/flexible-collision-library/fcl.git
    % mkdir fcl/build
    % cd fcl/build
    % cmake -DCMAKE_BUILD_TYPE=Release ..
    % make -j$(nproc)
    % sudo make install
    
Once all dependencies are installed, build with `catkin_make` as normal.

To enable release build:

    % catkin_make -DCMAKE_BUILD_TYPE=Release