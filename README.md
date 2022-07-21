# HKUST Smart Car Simulator for **Windows**

HKUST Smart Car Simulator is a set of software package for Intelligence Racing sub-team members in Robotics Team in HKUST. Members can use the tool to test their algorithm without having the car physically.

This is the documentation for the usage of the simulator.

## Installation

### Tools needed

1. C++ (for building your client side)
2. Python 3 (for hosting the simulation server)
3. Visual Studio Code (suggested) or any other code editor
4. Git (optional, for cloning and your development)

### Set-up

Clone this project with Git into your local machine or just directly download ZIP.

```bash
git clone https://github.com/BenBenCHAK/HKUST_SmartCarSimulator.git
```

Install [PyBullet](https://pybullet.org/wordpress/) for hosting 3D simulation server. Some other essential packages used are:
- [NumPy](https://numpy.org/install/) for providing an array storage of image
- [Sockets](https://pypi.org/project/sockets/) for client-server communication
- [PIL](https://pypi.org/project/Pillow/) for simple imaging purposes.

Install them if you have not installed before.

```bash
pip install pybullet
pip install numpy
pip install sockets
pip install Pillow
```

## Usage

### Basic Directory Structure

```
/
├── lib
│   ├── SClib.cpp
│   ├── SClib.h
│   └── smart_car_server.py
├── src
│   ├── simplecar.urdf
│   ├── simpleplane.urdf
│   ├── track.urdf
│   └── track_smaller.png
├── makefile
├── simulation.cpp
├── simulation.exe
└── smart_car.py
```

The "lib" directory contains the C++ library for the client side to get the image from the car in the server side and control the car as needed. The "simulation.cpp" is a **quickstart** demo code for the library's usage. As for the "makefile", it is a reference for you to compile the "simulation.cpp" into the "simulation.exe".

The "src" directory contains 3D materials and textures for the car simulation. To run the server, just run "smart_car.py", which includes the module "smart_car_server.py" inside.

### Starting both the server and client simultaneously

For the server side, move to the root and run:
```bash
python smart_car.py
```

For the client side, move to the root and run:
```bash
make
.\simulation.exe
```

> Hint: If you cannot "make", then just manually compile the C++ code. Refer to the makefile.

## To-do list
1. Client real time control
2. Sending camera image to client
3. Multi threading applied

<!-- ## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change. -->

## Authors and acknowledgment
Contact me at <whchak@connect.ust.hk> or find me directly if you find any bugs.

<!-- 
## License
[MIT](https://choosealicense.com/licenses/mit/) -->
