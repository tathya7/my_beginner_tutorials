# ROS2 - Beginner Tutorials - Part 2

This repository contains the five tutorial series of the ROS2 Beginner Tutorials. The branch `ros_services_logging_launch` contains the simple C++ publisher subscriber node as a part of assignment one. Further a service to modify the string was added and a launch file is created with a parameter to it. Made by **Tathya Bhatt** for the course *ENPM700-Software Development for Robotics*


## Dependencies

This project uses the ROS2 Humble distribution and assumed to be a dependency

## Building the Code

```
# Make a workspace directory
mkdir -p ~/ros2_ws/src

# Go to the directory and clone the repository
git clone 

#  Install resdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y

# Go back to the workspace folder and build the package
colcon build --packages-select beginner_tutorials

# After successful build, source the package
source install/setup.bash

# Launch both the nodes using the command
ros2 launch beginner_tutorials talker.launch.py

# To run the service, go to your workspace and run the below commmand
ros2 service call /change_string beginner_tutorials/srv/ChangeString "{new_string: 'MyNewString'}"


```

## Checking Google C++ Style Guide and static code analysis

```
# Go to the package directory and src folder
cd ~/ros_ws/src/beginner_tutorials/src

# Format and modify the code 
clang-format -style=Google -i subscriber__member_function.cpp
clang-format -style=Google -i publisher__member_function.cpp

# Go back to workspace directory and perform clang-tidy checks
cd ..
cd ..
cd ..
clang-tidy -p build/ src/beginner_tutorials/src/*.cpp

# Static code analysis
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order  src/*.cpp >  results/cpplint.txt

```
## Author
- **Name** : Tathya Bhatt
- **UID** : 120340246
