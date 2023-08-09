## Introduction
This repostiory is based on [MIT-Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software), we developed this repository which contains the Robot and Simulation software for the Mini-Cheetah project by NAVER LABS. 

## Dependencies
* Follwing dependencies, we've been tested on Ubuntu 20.04. 
```
sudo apt-get install freeglut3-dev
sudo apt-get install libblas-dev liblapack-dev
sudo apt install libeigen3-dev liblcm-dev liblcm1 liblcm-java
sudo apt-get install build-essential
sudo apt-get install mesa-common-dev libglu1-mesa-dev
sudo apt-get install libqt5gamepad5-dev
sudo apt install libglib2.0-dev
sudo apt install cmake 
```

* Set gcc, g++ 7.x version 
```
sudo apt install gcc-7 g++-7
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 10
sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 10
```

* Install LCM
```
sudo apt update && sudo apt install build-essential g++ libglib2.0-dev cmake
git clone https://github.com/lcm-proj/lcm.git 
cd lcm
mkdir build
cd build
cmake ..
make 
sudo make install
```

* Install Eigien
```
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
git checkout 3.3 
mkdir build 
cd build 
cmake ..
make 
sudo make install
```

* Install Qt-5.10.0 
Install Qt downloader on [Qt Group](https://www.qt.io/download), then select 5.10.0 version via Archive filter in downloader

## Build
To build all code:
```
mkdir build
cd build
cmake ..
./../scripts/make_types.sh
make -j4
```

## Run simulator
To run the simulator:
1. You need to setup the UDP Multicast route to use LCM on a single host
```
sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo
```
2. Open the control board
```
cd cheetah-os/build
./sim/sim
```
3. In the another command window, run the robot control code
```
./user/${controller_folder}/${controller_name} ${robot_name} ${target_system}
```
Example)
```
./user/Naverlabs_Controller/naverlabs_ctrl m s

```
3: Cheetah 3, m: Mini Cheetah
s: simulation, r: robot


