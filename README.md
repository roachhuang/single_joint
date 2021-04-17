
    to do:
      see how to reset cmd[] if no twist cmd 
      figure out if nh_ is really needed
      pub pos in read method. if have time (not important)
      tt motor is unstable, so just tune pid for a certain sweetspot speed.      
      to understand rates and queue size in each node to see if they are properly set.
      tick_meter param in 2 file - pid_velocity and diff_tf. make the param in rosparm
      fix serial_node wrong checksum issue

    
    mega 2560 int pins: 2, 3, 21, 20, 19, 18 (INT0~5)

    pid for uno: lwheel: 80, 0.007, 0.0
    rwheel: 110, 0.0032, 0.0

    note that we cannot run position controller and velocity controller at the same time, so we create two launch files.
    
    rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0

    test position & velocity:
    rosrun rqt_reconfigure rqt_reconfigure
    rosrun rqt_graph rqt_graph

    // velocity
    roslaunch diff_drive check_velocity_controller.launch
    rostopic pub /single_joint_actuator/joint1_velocity_controller/command std_msgs/Float64 "data: 0.37"
    
    // position
    roslaunch diff_drive check_position_controller.launch
    rostopic pub /single_joint_actuator/joint1_position_controller/command std_msgs/Float64 "data: 1.57"
    rosrun rqt_plot rqt_plot
    
    arduino:
      encAB

    rviz:
      fixed frame: world
      add RobotModel

    diagnostic:
      rosservice list
      rosservice call /joint_read true
    # rostopic pub /jointservo sensor_msgs/JointState '{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: ""}, name: ["art1"], position: [150.0], velocity: [0.0], effort: [0.0]}' --once
    rostopic pub /joint_states_from_arduion sensor_msgs/JointState '{position: [30], velocity: [40]}' --once

rosparam list
rosparam get <param key>
rqt_gui rq

 history | grep roslaunch
 history <number>
 # run previous cmd again
 !<number>

wheel_circumference = pi * d
wheel_rotations_per_meter = 1 / (pi * d)
ticks_per_meter = wheel_rotations_per_meter * ticks_per_rev
