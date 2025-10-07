# Running Motor Failure detection
---
## Running Classical failure Detection

Navigate inside the detection package and run the following command
```bash
bash detection.sh classical
```
Ensure that correction argument is being passed to `detection.sh`

In a new terminal run the following publisher command and replace `motor_number` with the motor number you intend to fail:
```bash
ros2 topic pub -r 1 /motor_failure/motor_number std_msgs/msg/Int32 "{data: motor_number}"
```
The File containig residuals will be stored in the current working directory under the name: *timeseries_residuals.csv*
### To plot the residuals run 
```bash
bash plot.sh
```
---
## Running ML based failure Detection

Navigate inside the detection package and run the following command
```bash
bash detection.sh ml
```
Ensure that correction argument is being passed to `detection.sh`

In a new terminal run the following publisher command and replace `motor_number` with the motor number you intend to fail:
```bash
ros2 topic pub -r 1 /motor_failure/motor_number std_msgs/msg/Int32 "{data: motor_number}"
```