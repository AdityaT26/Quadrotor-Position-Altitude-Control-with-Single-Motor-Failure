# Classical Motor Failure detection
---
## Summary

Upon running the motor failure detection node, the node outputs data like:
```
Initialising failure detection node
Initialised...

Motor 1 failed
Delay(ms): 59.619444
```
Correctly predicting which motor has failed.

### Robustness

The observer has a **100%** detection rate of motor failure in nominal conditions like high-speed linear traversal and near stationary yaw turns.

The observer is not yet adaptive, we aim to cascade a non-linear **Adaptive Thau Observer (ATO)** that learns the gain matrix on the fly and takes care of unmodelled system parameters.

### Challenges

The residual signs may not necessarily map to the same motor during agile flight like high speed banking turns or high-speed yaw rotation. An **ATO** is being worked on to mitigate this.