# needle_steering

## Install ROS2 Foxy

## Install OROCOS toolchain

This doesn't (shouldn't) need to be in your ROS workspace. So keep things organized and put that somewhere else. The following will do a system install. You can keep this in your account if you prefer.

### Install dependencies

```shell
sudo apt-get install liblua5.1-0-dev doxygen liblog4cxx-dev ros-foxy-test-msgs 
```

### Install Orocos Toolchain

```shell
mkdir -p ~/src/orocos
cd ~/src/orocos
git clone -b ros2 https://github.com/orocos-toolchain/orocos_toolchain.git --recursive
cd orocos_toolchain
git submodule foreach "git checkout ros2 || :"
sudo ./configure --prefix=/usr/local/orocos_toolchain
sudo make install
```
## Install Orocos ROS2 Integration
These are ROS packages and should go in your workspace
```shell
mkdir -p ~/ros/src
git clone git@github.com:orocos/rtt_ros2_integration.git
git clone git@github.com:simonleonard/needle_steering.git --recurssive
```
Build everything. The ROS test packages do not build on my system so the following command ignores them
```shell
colcon build --packages-ignore-regex rtt_ros2_idl_tests rtt_ros2_params_tests rtt_ros2_services_tests rtt_ros2_test_msgs rtt_ros2_tests rtt_ros2_topics_tests --cmake-args -DCMAKE_CXX_FLAGS=-DRTT_COMPONENT -DOROCOS-RTT_DIR=/usr/local/orocos_toolchain/lib/cmake/orocos-rtt
```
