#!/bin/bash

if [[ -z $1 ]];
then 
    echo "No parameter passed."
else
    if [[ $1 == "classical" ]];
    then
        echo "Running classical detection"
        python3 src/detection_classical/node.py
    elif [[ $1 == "ml" ]];
    then
        echo "Running ML detection"
        python3 src/detection_ml/node.py
    else
        echo "Invalid parameter passed."
    fi
fi