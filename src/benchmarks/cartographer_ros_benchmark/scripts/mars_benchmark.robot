#!/usr/bin/env -S lambkin robot -f

*** Settings ***
Resource          lambkin.resource

Test Template     Benchmark Cartographer ROS 2D SLAM
Suite Setup       Lambkin Setup
Suite Teardown    Benchmark Teardown

*** Test Cases ***       DATASET
Indoor Loop              MARS_Loop_1.bag MARS_Loop_2.bag MARS_Loop_3.bag

*** Keywords ***
Benchmark Cartographer ROS 2D SLAM
    [Arguments]  ${dataset}
    Register Parameters  dataset=${dataset}
    Use /tf /vertical_velodyne/velodyne_points /odometry/filtered data in ${dataset} at 5x as input
    Track /tf:odom.base_link /tf:map.base_link trajectories
    And save the resulting map
    Use mars_benchmark.launch in cartographer_ros_benchmark package to launch
    Use a sampling rate of 20 Hz to track computational performance
    Benchmark Cartographer ROS for 10 iterations

Benchmark Teardown
    Lambkin Teardown
    Run Keyword If All Tests Passed
    ...  Generate report using mars_report in cartographer_ros_benchmark package

