#!/bin/sh

# some basic settings
export TARGETDIR=/media/external #this is where ROS will be installed to, see below
export ROS_ROOT=$TARGETDIR/ros/ros # ROS_ROOT, as usual
export ROS_PACKAGE_PATH=$ROS_ROOT/../stacks # ROS_PACKAGE_PATH, as usual
export CTC_DIR=/opt/aldebaran/nao-cross-toolchain # version 1.10
export ROS_DEPS=$ROS_ROOT/../ros-deps/
export CTC_USR_DIR=$CTC_DIR/staging/geode-linux/usr/
export CTC_BIN_DIR=$CTC_DIR/cross/geode/bin
export PYTHONPATH=$ROS_ROOT/core/roslib/src
export CPATH=$ROS_ROOT/../ros-deps/include/
export GEODE_CXX=$CTC_DIR/cross/geode/bin/i586-linux-g++
export GEODE_CC=$CTC_DIR/cross/geode/bin/i586-linux-gcc
export PATH=$CTC_BIN_DIR:$ROS_ROOT/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export LD_LIBRARY_PATH=.

# automake compiler/linker flags
export LDFLAGS="-Wl,--sysroot,$CTC_DIR/staging/geode-linux/ -lgcc -L$CTC_DIR/cross/geode/i586-linux/lib/ -L$CTC_DIR/staging/geode-linux/usr/lib  -lc -lstdc++ -ldl"
export CPPFLAGS="--sysroot /$CTC_DIR/staging/geode-linux/ -I/$CTC_DIR/staging/geode-linux/usr/include/ -I/$CTC_DIR/cross/geode/lib/gcc/i586-linux/4.3.3/include/ -I/$CTC_DIR/staging/geode-linux/usr/include/c++/ -I/$CTC_DIR/staging/geode-linux/usr/include/c++/i586-linux/ -I/$CTC_DIR/cross/geode/i586-linux/include/c++/ -I/$CTC_DIR/cross/geode/i586-linux/include/c++/backward/ -I/$CTC_DIR/cross/geode/i586-linux/include/c++/i586-linux -I/$CTC_DIR/cross/geode/i586-linux/include/ -I/$CTC_DIR/staging/geode-linux/usr/lib"
export CFLAGS="-march=geode"
export CXXFLAGS="-march=geode"

source $TARGETDIR/ros/diamondback/setup.sh
