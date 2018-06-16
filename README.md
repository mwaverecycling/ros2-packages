# MWAVE ROS 2 Packages

Assorted packages for ROS 2 utilized by MWave Recycling

## Dependencies

In order to build and run, you must install and setup [ROS 2](https://github.com/ros2/ros2/wiki/Installation)

The [i2cbridge](https://github.com/mwaverecycling/ros2-packages/tree/master/src/mwave/i2cbridge) package depends upon the [I2CPP Library](https://github.com/mwaverecycling/I2CPP) for interfacing with I2C

## Build

First ensure ROS 2 is in your path, then build the ros2-packages
```bash
# change into your ros2 workspace and source the install
# if you don't know where it is, look in your /opt directory
cd ros2_ws/
source install/local_setup.bash

# now build the repository and source it
cd ros2-packages
ament build
source install/local_setup.bash
```
