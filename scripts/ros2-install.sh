#!/bin/bash

# From https://github.com/ros2/ros2/wiki/Linux-Development-Setup

progname=$(basename $0)
subcommand=$1


check_locales() {
    echo "Checking locale..."
    if [ "$LANG" != "en_US.UTF-8" ]; then
        echo "   LANG: $LANG != en_US.UTF-8. Generating locale..."
        sudo locale-gen en_US en_US.UTF-8
        sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
        export LANG=en_US.UTF-8
    else
        echo "   Locale $LANG good!"
    fi
}

check_repos() {
    echo "Updating repo list..."
    if [ -e "/etc/apt/sources.list.d/ros-latest.list" ]; then
        echo "   Repo checks out!"
    else
        command -v lsb_release >/dev/null 2>&1 || { echo >&2 "lsb_release required but not installed. Aborting."; exit 1; }
        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116 \
            || sudo apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116
    fi
}

install_deps() {
    echo "Installing dependencies..."
    sudo apt-get update && \
    sudo apt-get install git wget && \
    sudo apt-get install build-essential cppcheck cmake libopencv-dev python-empy python3-dev python3-empy python3-nose python3-pip python3-pyparsing python3-setuptools python3-vcstool python3-yaml libtinyxml-dev libeigen3-dev && \
    sudo apt-get install clang-format pydocstyle pyflakes python3-coverage python3-mock python3-pep8 uncrustify && \
    sudo pip3 install flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pytest pytest-cov pytest-runner && \
    sudo apt-get install libasio-dev libtinyxml2-dev || \
    { echo "   Error installing dependencies!"; exit 1; }
    echo "Dependencies installed!"
}

make_workspace() {
    echo "Setting up workspace..."
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
    vcs-import src < ros2.repos
    echo "   Workspace set up!"
}

choose_packages() {
    cd ~/ros2_ws/src/ros2
    echo "Searching packages..."
    for pdir in ./* ; do
        judge_directory $pdir
    done
}
judge_directory() {
    dname=$(basename $1)
    echo -n "   Would you like to ignore '$dname'? [y/n/d] "
    read act
    case $act in
        "y")
            touch "$1/AMENT_IGNORE"
            ;;
        "d")
            for cdir in "$1/*" ; do
                judge_directory $cdir
            done
            ;;
    esac
}


install_ros2() {
    check_locales
    check_repos
    install_deps
    make_workspace
    choose_packages
}

case $subcommand in
    ""|"download")
        install_ros2
        ;;
    "choose")
        choose_packages
        ;;
    "build")
        cd ~/ros2_ws
        export MAKEFLAGS=-j1
        ./src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install
        ;;
esac
