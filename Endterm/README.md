# Team 30 ideaForge End Term Report
## Tables of Content
 - [Setup](#setup)
 - [Running Motor Failure Detection Methods](#running-motor-failure-detection-methods)
 - [Running Fault Tolerant Controller in Simulated Ideal Dynamics](#running-fault-tolerant-controller-in-simulated-ideal-dynamics)
 - [Running Fault Tolerant Controller in Gazebo Simulation after integratoin with PX4](#running-fault-tolerant-controller-in-gazebo-simulation-after-integratoin-with-px4)
 - [Gazebo Simulation Videos](#gazebo-simulation-videos)
# Setup
## Workspace Setup
The Steps for Docker Container Setup is detailed at [Workspace Setup](setup/setup.md) with **Dockerfile** present at [Dockerfile](setup/Dockerfile)
## PX4 Motor Failure Module Setup for Gazebo Classic
After setting up the Docker Container, do the following steps:
* Add this snippet to *~/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml* to **publications**
```yaml
  - topic: /fmu/out/event
    type: px4_msgs::msg::Event
```
* Add this line to *~/PX4-Autopilot/boards/px4/sitl/default.px4board*
```config
MODULES_MOTOR_FAILURE_DETECTION=y
```
* Copy the Directory *files/motor_failure_detection* inside the Docker Container to *~/PX4-Autopilot/src/modules/*
* Clean and Build PX4 SITL Gazebo
```bash
cd ~/PX4-Autopilot
make clean
make px4_sitl
```

## Running PX4 SITL Gazebo Classic with motor_failure_detection Module
* Run PX4-Autopilot in Gazebo Classic
```bash
cd ~/PX4-Autopilot/ && make px4_sitl gazebo-classic 
```
* Run MAVROS Proxy
```bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@localhost:14557"
```
* Run uXRCE-DDS middleware
```bash
MicroXRCEAgent udp4 -p 8888
```
Note: If the **mavros** command fails, free some resources (CPU & RAM) on your PC & Try again in 1 minute. And repeat this setup until it stops crashing.<br />
You should see a Drone in the Ground.<br />
Inside the px4 shell, you've to start the **motor_failure_detection** Module by entering the following command
```bash
px4> motor_failure_detection
```
It will start publishing to **/fmu/out/event** uORB Topic.<br />
The Message used for **/fmu/out/event** is **px4_msgs/msg/Event.msg** and it's structure is as follows
```
# Events interface
uint64 timestamp			# time since system start (microseconds)

uint32 id                   # Event ID
uint16 event_sequence       # Event sequence number
uint8[25] arguments         # (optional) arguments, depend on event id

uint8 log_levels            # Log levels: 4 bits MSB: internal, 4 bits LSB: external

uint8 ORB_QUEUE_LENGTH = 16
```
**arguments[0]** contain the information about motor_failure. When no motor failure is detected, it's value is 0.
Copy the Directory *files/detection* inside the Docker Container to  *~/ros2_ws/src/*<br />
# Running Motor Failure Detection Methods
The usage for the Detection Nodes can be found at [README.md](files/detection/README.md)
# Running Fault Tolerant Controller in Simulated Ideal Dynamics
The usage of the implemented Fault Tolerant NMPC Controller can be found at [README.md](files/NMPC_Controller/README.md)
# Running Fault Tolerant Controller in Gazebo Simulation after integratoin with PX4
Run the following Commands in different terminals
```bash
PX4_GZ_MODEL_POSE="0,0,0,0,0,1.57" ~/PX4-Autopilot/build/px4_sitl_default/bin/px4
ros2 launch mavros px4.launch fcu_url:="udp://:14540@localhost:14557"
MicroXRCEAgent udp4 -p 8888
```
Run [Controller](files/Controller/main.py)
```
python3 main.py
```
# Gazebo Simulation Videos
## Classical Approach for Motor Failure Detection
<video src="assets/videos/classical_detection.mp4">Classical Approach</video>

## Hovering
<video src="assets/videos/hovering.mp4">Hovering</video>

## Landing
<video src="assets/videos/landing.mp4">Landing</video>

## Return to Home (RTH)
<video src="assets/videos/return_to_home.mp4">Return to Home</video>