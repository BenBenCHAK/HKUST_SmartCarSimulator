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

Install [PyBullet](https://pybullet.org/wordpress/) for hosting the 3D simulation server. Some other essential packages used are:
- [NumPy](https://numpy.org/install/) for providing an array storage of image
- [Sockets](https://pypi.org/project/sockets/) for client-server communication
- [PIL](https://pypi.org/project/Pillow/) for simple imaging purposes.

Install them if you have not installed before.

```bash
pip install numpy
pip install sockets
pip install pybullet
pip install pillow
```

## Usage

### Basic Directory Structure

```
/
├── lib
│   ├── sdl
│   │    └── ...
│   ├── SClib.cpp
│   ├── SClib.h
│   └── smart_car_server.py
├── src
│   ├── simplecar.urdf
│   ├── simpleplane.urdf
│   └── track.obj
├── .gitignore
├── SDL2.dll
├── makefile
├── simulation.cpp
├── simulation.exe
├── simulation_gui.cpp
├── simulation_gui.exe
└── smart_car.py
```

The "lib" directory contains both external C++ and Python libraries for the client side to get the image from the car in the server side and control the car as needed.

The "src" directory contains 3D materials and textures for the car simulation.

In the root directory:
- "smart_car.py" is the server file that you need to start for simulation;
- "simulation.cpp" is a **quickstart** demo code for the library's usage, so feel free to **modify**;
- "simulation_gui.cpp" is an optional tester code for testing if the images are correctly received, with the aid of the SDL2 library;
- "makefile" helps you compile the C++ source codes into executables, so **modify** it if you need;
- "SDL2.dll" is an essential DLL file for the SDL library;
- ".gitignore" is a file to tell git to ignore certain files / directories.

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

### PyBullet server usage

The GUI provided looks like this:

![PyBullet GUI](readme_img/initial_gui.png)

> You can manually adjust your view in God Mode. To move around, press the arrow keys. To look around, hold any CTRL key and the left mouse button. To zoom in or out, scroll the mouse wheel or hold any CTRL key and the right mouse button.

There are two modes for our simulation. The God Mode is for freestyle and manual controlling the car in real-time; the Client Mode is for transmission of command and car control in real-time. The panel on the right is for controlling parameters for the simulation.

### Right-side Params Panel usage

#### God Mode

![God Mode Parameters](readme_img/god_mode_params.png)

1. Button to switch to Client Mode
2. Button to remove the current trajectory if there is any
3. Slider to adjust the car's x-coordinate before simulation
4. Slider to adjust the car's y-coordinate before simulation
5. Slider to adjust the car's direction before simulation
6. Slider to adjust the turning of the car during simulation
7. Slider to adjust the speed of the car during simulation
8. Button to start or end simulation

#### Client Mode

![Client Mode Parameters](readme_img/client_mode_params.png)

1. Button to switch to God Mode
2. Slider to adjust the camera's height before simulation
3. Slider to adjust the camera's offset from the front of the car before simulation
4. Slider to adjust the camera's angle before simulation
5. Button to leave or hide the car's trajectory during simulation
6. Button to save a picture locally from the camera
7. Button to start or end simulation

## To-do list 
- **code refactoring and beautifying**
- car mass distribution, friction
- timer
- first pov control (need disable w and s keys)
- only control front wheel for turn, back for speed
- hoist all constant, reset button to user-stored, save all customizable value to csv (np.savetxt): like a config file
- client check if lost connection before connecting again (doing)
- camera projection view (i.e. fish eye stuff)
- really moving car camera
- gradual change of motor speed
- noise
- different tracks and cars to choose
- new command: reset simulation, this maybe useful for deep learning
- PID (PD Control as in setJointMotorControl2)
- multiple clients in same server
- continue commands if server down but client not down

## Q&A

1. Q: How to solve this error: "TypeError: tuple indices must be integers or slices, not tuple"? A: Install PyBullet after installing NumPy.

<!-- ## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change. -->

## Authors and acknowledgment
Contact me at <whchak@connect.ust.hk> or find me directly if you find any bugs or need any aids.

<!-- 
## License
[MIT](https://choosealicense.com/licenses/mit/) -->
