# ROS2 - Beginner Tutorials - Part 2

This repository contains the five tutorial series of the ROS2 Beginner Tutorials. The branch `ros_tf2_unitTest_bagFiles` contains the modified code including the Level 2 Integration Test, the recorded bag file and the tf2_transform broadcast. Made by **Tathya Bhatt** for the course *ENPM700-Software Development for Robotics*


## Dependencies

This project uses the ROS2 Humble distribution.

## Building & Running Steps

- ### Cloning the package into a workspace folder:
```
# Make a workspace directory
mkdir -p ~/ros2_ws/src

# Go to the directory and clone the repository
git clone https://github.com/tathya7/my_beginner_tutorials.git

#  Install resdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y
```
- ### Build the package and source:
```
# Go back to the workspace folder and build the package
colcon build --packages-select beginner_tutorials

# After successful build, source the package
source install/setup.bash

```

- ### Run the publisher node:
```
ros2 run beginner_tutorials talker
```

- ### Run the subscriber node:
```
ros2 run beginner_tutorials listener
```

- ### Call the service using ROS2 Command Line Tool:
```
ros2 service call /change_string beginner_tutorials/srv ChangeString "{new_string: 'MyNewString'}"
```

- ### Use launch file to start the publisher and record a rosbag using:
    The bag will be saved in your root of the workspace in a folder called "my_bag".
```
ros2 launch beginner_tutorials talker.launch.py record_bag:=1

```
- ### To check if the bag files have been recorded:
```
cd my_bag/
ros2 bag info my_bag_0.db3

```
- ### Playing the recorded bag:
```
# Run the subscriber in a new terminal
ros2 run beginner_tutorials listener

# In another terminal, go the bag directory and play the bag
ros2 bag play my_bag_0.db3
```
- ### To run the integration test:
```
cd ros2_ws/
colcon test  --return-code-on-test-failure --event-handlers console_cohesion+
```

- ### Catch2 Test Output Log
```
1: Test command: /home/tathyab/anaconda3/bin/python3 "-u" "/opt/ros/humble/share/catch_ros2/cmake/../scripts/run_test.py" "/home/tathyab/my_beginner_tutorials/build/beginner_tutorials/test_results/beginner_tutorials/ExampleIntegration_TestPython.xml" "--package-name" "beginner_tutorials" "--command" "ros2" "launch" "beginner_tutorials" "test.launch.py" "result_file:=/home/tathyab/my_beginner_tutorials/build/beginner_tutorials/test_results/beginner_tutorials/ExampleIntegration_TestPython.xml"
1: Test timeout computed to be: 60
1: -- run_test.py: invoking following command in '/home/tathyab/my_beginner_tutorials/build/beginner_tutorials':
1:  - ros2 launch beginner_tutorials test.launch.py result_file:=/home/tathyab/my_beginner_tutorials/build/beginner_tutorials/test_results/beginner_tutorials/ExampleIntegration_TestPython.xml
1: [INFO] [launch]: All log files can be found below /home/tathyab/.ros/log/2024-11-13-18-13-29-214052-tathyab-24112
1: [INFO] [launch]: Default logging verbosity is set to INFO
1: [INFO] [talker-1]: process started with pid [24114]
1: [INFO] [integration_test-2]: process started with pid [24116]
1: [integration_test-2] [INFO] [1731539609.302815530] [Node1]: Got duration=2
1: [integration_test-2] [INFO] [1731539609.303599351] [Node1]: duration = 2.153e-06 timeout=2
1: [talker-1] [WARN] [1731539609.304955472] [text_publisher]: Changing publish frequency
1: [talker-1] [FATAL] [1731539609.306311003] [text_publisher]: This is a example fatal message
1: [talker-1] [INFO] [1731539609.606516405] [text_publisher]: Publishing: This is assignment 1 0
1: [integration_test-2] [INFO] [1731539609.703962999] [Node1]: I heard:This is assignment 1 0
1: [integration_test-2] [INFO] [1731539609.803740550] [Node1]: duration = 0.500101 got_topic=1
1: [integration_test-2] Randomness seeded to: 960784078
1: [integration_test-2] ===============================================================================
1: [integration_test-2] All tests passed (1 assertion in 1 test case)
1: [integration_test-2] 
1: [talker-1] [INFO] [1731539609.906506765] [text_publisher]: Publishing: This is assignment 1 1
1: [INFO] [integration_test-2]: process has finished cleanly [pid 24116]
1: [INFO] [launch]: process[integration_test-2] was required: shutting down launched system
1: [INFO] [talker-1]: sending signal 'SIGINT' to process[talker-1]
1: [talker-1] [INFO] [1731539609.931656485] [rclcpp]: signal_handler(signum=2)
1: [INFO] [talker-1]: process has finished cleanly [pid 24114]
1: -- run_test.py: return code 0
1: -- run_test.py: verify result file '/home/tathyab/my_beginner_tutorials/build/beginner_tutorials/test_results/beginner_tutorials/ExampleIntegration_TestPython.xml'
1/1 Test #1: ExampleIntegration_TestPython ....   Passed    1.22 sec

100% tests passed, 0 tests failed out of 1

Total Test time (real) =   1.22 sec


```
## Checking Google C++ Style Guide and static code analysis

```
# Go to the package directory and src folder
cd ~/ros_ws/src/beginner_tutorials/src

# Format and modify the code 
clang-format -style=Google -i subscriber_member_function.cpp
clang-format -style=Google -i publisher_member_function.cpp

# Go back to workspace directory and perform clang-tidy checks
cd ..
cd ..
cd ..
clang-tidy -p build/ src/beginner_tutorials/src/*.cpp

# Static code analysis, go to the package directory 
cd src/beginner_tutorials
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order  src/*.cpp > cpplint.txt

```
## Author
- **Name** : Tathya Bhatt
- **UID** : 120340246
