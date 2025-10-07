#!/usr/bin/python
# -*- coding: utf-8 -*-

import rclpy
import sys
from time import time
from pathlib import Path
import numpy as np

from observer import Observer
from q_to_rpy import quaternion_to_euler
from rpy_rotation import transform_rpy
from params import J, l, J_r

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from math import sqrt

from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32
from px4_msgs.msg import VehicleAttitude, ActuatorMotors

import csv


cwd = Path.cwd()

def append_data(filename, data):
    with open(filename, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(data)

class Res_Gen(Node):

    def __init__(self, observer):
        print("Initialising failure detection node")
        self.file = open(f"./timeseries_residuals.csv", 'w')
        self.failure_time = 0
        self.observer = observer
        super().__init__('classical_detection')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.imu_subscription = Subscriber( self, Imu, '/mavros/imu/data', qos_profile=qos_profile)
        self.motor_subscription = self.create_subscription(Int32, '/motor_failure/motor_number', self.update_time, qos_profile=qos_profile)
        self.vehicle_attitude_subscription = Subscriber(self, VehicleAttitude, '/fmu/out/vehicle_attitude', qos_profile=qos_profile)
        self.actuator_motors_subscription = Subscriber(self, ActuatorMotors, '/fmu/out/actuator_motors', qos_profile=qos_profile)

        # Approximate Time synchronizer to sync messgaes
        self.sync = ApproximateTimeSynchronizer([
            self.imu_subscription,
            self.vehicle_attitude_subscription,
            self.actuator_motors_subscription,
            ], 15, 0.05, allow_headerless=True)

        # register a callback with synchronizer
        self.sync.registerCallback(self.synccallback)
        print("Initialised...")

    def update_time(self, msg):
        self.failure_time=self.get_clock().now().nanoseconds
        
    def synccallback(
        self,
        imu_data,
        att_data,
        act_motors_data,
        ):
        phi_dot = imu_data.angular_velocity.x
        theta_dot = imu_data.angular_velocity.y
        psi_dot = imu_data.angular_velocity.z
        rpm = [1000 * sqrt(act_motors_data.control[0]),
               1000 * sqrt(act_motors_data.control[1]),
               1000 * sqrt(act_motors_data.control[2]),
               1000 * sqrt(act_motors_data.control[3])]

        #reorder rpm to match the order of the motors in clockwise order from top right
        rpm_renumbered = [rpm[0], rpm[3], rpm[1], rpm[2]] #real_motor_velocity in clockwise order from top right
        
        #Residual Angular Velocity
        omega_r = -rpm_renumbered[0] + rpm_renumbered[1] - rpm_renumbered[2] + rpm_renumbered[3]
        
        #get RPY from quaternion
        (phi, theta, psi) = quaternion_to_euler(att_data.q)
        
        #calculate the torque from the motor setpoints (they are normalised rpm setpoints)
        rpm_torque = [0,0,0]
        rpm_torque[0]=(act_motors_data.control[2]**2+act_motors_data.control[1]**2-act_motors_data.control[3]**2-act_motors_data.control[0]**2)*(1000*5.84e-6)*(0.24703/sqrt(2)) 
        rpm_torque[1]=(act_motors_data.control[1]**2+act_motors_data.control[3]**2-act_motors_data.control[0]**2-act_motors_data.control[2]**2)*(1000*5.84e-6)*(0.24703/sqrt(2))
        rpm_torque[2]=(act_motors_data.control[0]**2+act_motors_data.control[1]**2-act_motors_data.control[2]**2-act_motors_data.control[3]**2)*(1000*0.6e-6)
        rpm_torque = np.reshape(rpm_torque,(3,1))
        x = np.array([
        [phi],
        [theta],
        [psi],
        [phi_dot],
        [theta_dot],
        [psi_dot],
        ])
        
        #Getting Residuals from observer
        residual = self.observer.get_residual(x, omega_r, rpm_torque)
        
        #rotate residual 45 degrees
        residual = transform_rpy(roll=residual[0][0], pitch=residual[1][0], yaw=residual[2][0])
        append_data("./timeseries_residuals.csv",[self.get_clock().now().nanoseconds, residual[0], residual[1], residual[2]])

        #Get Failed motor number from sign of residuals if they cross the threshold
        if(residual[1]>=0.01 or residual[0]<-0.01):
            detection_time=self.get_clock().now().nanoseconds #detect the time of failure
            if(residual[2]<0):
                print("motor 1 has failed")
                print(f'Delay(ms): {(detection_time-self.failure_time)*(1e-6)}') #delay in milliseconds
            else:
                print("motor 4 has failed")
                print(f'Delay(ms): {(detection_time-self.failure_time)*(1e-6)}') #delay in milliseconds
            self.destroy_node()
            rclpy.shutdown()
        elif(residual[0]>=0.01 or residual[1]<-0.01):
            detection_time=self.get_clock().now().nanoseconds #detect the time of failure
            if(residual[2]<0):
                print("motor 2 has failed")
                print(f'Delay(ms): {(detection_time-self.failure_time)*(1e-6)}') #delay in milliseconds
            else:
                print("motor 3 has failed")
                print(f'Delay(ms): {(detection_time-self.failure_time)*(1e-6)}') #delay in milliseconds
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    observer = Observer(J, l, J_r)
    residual_generator = Res_Gen(observer)
    rclpy.spin(residual_generator)
    residual_generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
