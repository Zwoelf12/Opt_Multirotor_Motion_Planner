# Opt_Multirotor_Motion_Planner
This repository includes an Optimization-Based Multirotor Motion Planner using Sequential Convex Programming (SCP). As SCP Algorithm SCvx is implemented following [Malyuta et. al. 2021](https://arxiv.org/abs/2106.09125).

tested on Ubuntu 20.04. with Python 3.9

## Installation:

Clone repository:
```
git clone https://github.com/Zwoelf12/Opt_Multirotor_Motion_Planner.git
```
Install Python version, pip and numpy:
```
sudo apt-get install python3.9 python3.9-dev python3.9-numpy python3.9-pip python3.9-distutils
```
If not already done add `./local/bin` to path: 
```
echo 'export PATH="${PATH}:$HOME/.local/bin"' >> ~/.bashrc
```
Install flexible collision library (fcl) for collision checking:
```
sudo apt-get install liboctomap-dev libfcl-dev
```
Install rowan for quaternion operations:
```
python3.9 -m pip install rowan
```
Install Python bindings for fcl:
```
python3.9 -m pip install python-fcl
```
Install CVXPY to handle convex subproblems:
```
python3.9 -m pip install cvxpy
```
Install JAX for automatic differentiation to obtain gradients:
```
python3.9 -m pip install jaxlib
python3.9 -m pip install jax
```
Install matplotlib and meshcat for visualization (perhaps you have to update Pillow) :
```
python3.9 -m pip install matplotlib
python3.9 -m pip install -U Pillow
python3.9 -m pip install meshcat
```
