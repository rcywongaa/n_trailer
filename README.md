# Setup

1. Download latest binary from <https://drake.mit.edu/from_binary.html>
1. Extract to `/opt/` directory

       sudo tar -xf drake-latest-bionic.tar.gz -C /opt/

1. Build

       mkdir build
       cmake -DCMAKE_PREFIX_PATH=/opt/drake ..
       make

1. Run

       /opt/drake/bin/drake-visualizer&
       ./bin/car

Currently fails due to <https://github.com/RobotLocomotion/drake/pull/9534>
