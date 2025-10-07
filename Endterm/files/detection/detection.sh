#!/bin/bash

if [[ -z $1 ]];
then 
    echo "No parameter passed."
else
    if [[ $1 == "classical" ]];
    then
        echo "Running classical detection"
        sudo LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH PYTHONPATH=/home/ideaForge/ros2_ws/install/message_filters/local/lib/python3.10/dist-packages:/home/ideaForge/ros2_ws/install/px4_msgs/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:/home/ideaForge/.local/lib/python3.10/site-packages python3 src/detection_classical/node.py
    elif [[ $1 == "ml" ]];
    then
        echo "Running ML detection"
        python3 src/detection_ml/node.py
    else
        echo "Invalid parameter passed."
    fi
fi