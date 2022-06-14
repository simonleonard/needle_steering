# needle_steering

## Install ROS2 Foxy

## Install OROCOS toolchain

This doesn't (shouldn't) need to be in your ROS workspace. So keep things organized and put that somewhere else. The following will do a system install. You can keep this in your account if you prefer.

Install dependencies

```shell
sudo apt-get install liblua5.1-0-dev doxygen liblog4cxx-dev ros-foxy-test-msgs 
```

Install Orocos Toolchain

```shell
mkdir -p ~/src/orocos
cd ~/src/orocos
git clone -b ros2 https://github.com/orocos-toolchain/orocos_toolchain.git --recursive
cd orocos_toolchain
git submodule foreach "git checkout ros2 || :"
sudo ./configure --prefix=/usr/local/orocos_toolchain
sudo make install
```
