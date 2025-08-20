Demonstration video : https://drive.google.com/file/d/1RA1LSaSnBDowe-eX10Sy_rbYIhmTfQ85/view?usp=sharing


1. Go to workspace:

    cd ~/ros2_ws

2. Source ROS 2 and workspace environment:


    colcon build
    source /opt/ros/humble/setup.bash
    source install/setup.bash


3. Launch Turtle system:

    ros2 launch turtle_control bringup.launch.py


4. Toggle service start:

    in a new terminal--

    cd ~/ros2_ws
    source install/setup.bash
    ros2 run turtlesim turtlesim_node

5. turn the pen off or on:

    In a new terminal--
    
    cd ~/ros2_ws
    source install/setup.bash

    ros2 service call /toggle_trace std_srvs/srv/SetBool "{data: false}"  (PEN OFF)

    ros2 service call /toggle_trace std_srvs/srv/SetBool "{data: true}"   (PEN ON)
