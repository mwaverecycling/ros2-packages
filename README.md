# MWAVE ROS2 Packages

Assorted packages for ROS2 utilized by MWave Recycling

## Dependencies
Install the i2cpp libraries by first cloning the repo.
```git clone https://github.com/mwaverecycling/I2CPP.git```
Then, `cd` into it.
```cd i2cpp/```
Make the libraries with:
```cmake .```
```sudo make install```

## Install
To use this code you will have to clone the repo with:
```git clone https://github.com/mwaverecycling/ros2-packages.git```
Source your installation of ros2 (mines at /opt/ros2_ws/...)
```source /opt/ros2_ws/install/local_setup.bash```
Then, build the project with ament.
```ament build```
