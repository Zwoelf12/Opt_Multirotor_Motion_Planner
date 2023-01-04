# Opt_Multirotor_Motion_Planner
This repository includes the Optimization-Based Multirotor Motion Planner. At the moment Sequential Convex Programming (SCP in this case SCvx) and K-Order Markov Optimization can be used as optimization methods.

tested on Ubuntu 20.04. with python 3.9

Clone repository:
```
git clone --recursive https://github.com/Zwoelf12/Opt_Multirotor_Motion_Planner.git
cd Opt_Multirotor_Motion_Planner
```
Install all necessary packages used in RAI:
```
# The following two commands depend on the config.mk -- see below
sudo apt-get update
make -j1 printUbuntuAll    # for your information: what the next step will install
make -j1 installUbuntuAll  # calls sudo apt-get install; you can always interrupt
```
Install necessary Python packages and pybind11 necessary for python bindings (RAI is originally a C++ library, but python bindings are provided):
```
echo 'export PATH="${PATH}:$HOME/.local/bin"' >> ~/.bashrc   #add this to your .bashrc, if not done already
sudo apt-get install python3 python3-dev python3-numpy python3-pip python3-distutils
pip3 install --user --upgrade pip
pip3 install --user pybind11
```
Compile using cmake. (Use ccmake to configure options, such as linking to bullet):
```
mkdir build
cd build
cmake ..
make -j $(command nproc)
```
