#! /bin/bash

# Executables
ROBOT_CONTROLLER_EXECUTABLE=roboscoop/robot_controller/EIFGENs/robot_controller/W_code/robot_controller
PATH_PLANNER_EXECUTABLE=roboscoop/path_planner/EIFGENs/path_planner/W_code/path_planner
MISSION_PLANNER_EXECUTABLE=roboscoop/mission_planner/EIFGENs/mission_planner/W_code/mission_planner

# Arguments
ROBOT_CONTROLLER_ARGS_DIR=roboscoop/robot_controller/execution_parameters_files
ROBOT_CONTROLLER_ARGS="$ROBOT_CONTROLLER_ARGS_DIR/files_parameters.txt"

PATH_PLANNER_ARGS_DIR=roboscoop/path_planner/execution_parameters_files
PATH_PLANNER_ARGS="$PATH_PLANNER_ARGS_DIR/path_planner_params.txt $PATH_PLANNER_ARGS_DIR/map_params.txt $PATH_PLANNER_ARGS_DIR/path_planner_topics.txt"

MISSION_PLANNER_ARGS_DIR=roboscoop/mission_planner/execution_parameters_files
MISSION_PLANNER_ARGS="$MISSION_PLANNER_ARGS_DIR/mission_planner_parameters.txt $MISSION_PLANNER_ARGS_DIR/mission_planner_topics.txt"

intexit() {
    # Kill all subprocesses (all processes in the current process group)
    kill -HUP -$$
}

hupexit() {
    # HUP'd (probably by intexit)
    echo
    echo "Interrupted"
    exit
}

trap hupexit HUP
trap intexit INT

$ROS_LAUNCH_COMMAND &

sleep 1

$ROBOT_CONTROLLER_EXECUTABLE $ROBOT_CONTROLLER_ARGS &

$PATH_PLANNER_EXECUTABLE $PATH_PLANNER_ARGS &

$MISSION_PLANNER_EXECUTABLE $MISSION_PLANNER_ARGS &

wait
