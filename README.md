# ROS2 - Beginner Tutorials - Part 1

This repository contains the five tutorial series of the ROS2 Beginner Tutorials. The branch `ros_pub_sub` contains the simple C++ publisher subscriber node as a part of assignment one. Made by **Tathya Bhatt** for the course *ENPM700-Software Development for Robotics*


## Dependencies

This project uses the ROS2 Humble distribution and assumed to be a dependency

## Building the Code

```
 $ # Make a workspace directory
mkdir -p ~/ros2_ws/src

# Go to the directory and clone the repository
git clone 

#  Install resdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y

# Go back to the workspace folder and build the package
colcon build --packages-select beginner_tutorials --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1

# After successful build, source the package
source install/setup.bash

# Run the publisher in terminal1
ros2 run beginner_tutorials talker

# Run the subscriber in terminal 2
ros2 run beginner_tutorials listener

```

## Checking Google C++ Style Guide and static code analysis

```
# Go to the package directory and src folder
cd ~/ros_ws/src/beginner_tutorials/src

# Format and modify the code 
clang-format -style=Google -i subscriber__member_function.cpp
clang-format -style=Google -i publisher__member_function.cpp

# Go back to package directory and perform clang-tidy checks
cd ..
clang-tidy -p build/ src/beginner_tutorials/src/*.cpp

# Static code analysis
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order  src/*.cpp >  results/cpplint.txt

```
## Author
- **Name** : Tathya Bhatt
- **UID** : 120340246
