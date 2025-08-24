The ur3 and ur3e controls command the joint velocities of their associated manipulators via the Universal_Robots_ROS_Driver. To build them, run the following commands in the terminal:
```bash
cd ur3 # cd ur3e
mkdir -p build && cd build
cmake ..
make
```
The gen3 control commands the cartesian twist of its associated manipulator via the ros_kortex driver. To build it, run the following commands in the terminal:
```bash
cd gen3
mkdir -p src && mv control src
catkin_make
```
