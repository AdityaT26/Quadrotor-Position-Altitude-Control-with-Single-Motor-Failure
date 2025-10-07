#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, OffboardControlMode, VehicleControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleAttitudeSetpoint, VehicleLocalPositionSetpoint, VehicleGlobalPosition, ActuatorMotors, VehicleThrustSetpoint, VehicleOdometry

import numpy as np
import matplotlib

from quadrotor import Quadrotor3D
from controller import Controller

iter = 0
matplotlib.use('TkAgg')

class Offboard(Node):
	def __init__(self):
		super().__init__("controller")

		# QOS Profiles
		qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST, depth=1)
		qos_profile_mavros = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

		# Publishers
		self.offboard_mode_publisher = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
		self.vehicle_command_publisher = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)
		self.actuator_motors_publisher = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_motors", qos_profile)
		self.vehicle_thrust_publisher = self.create_publisher(VehicleThrustSetpoint, "/fmu/in/vehicle_thrust_setpoint", qos_profile)
		self.trajectory_publisher = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_profile)

		# Subscribers
		## PX4
		self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile)
		self.status_subscriber = self.create_subscription(VehicleControlMode, "/fmu/out/vehicle_control_mode", self.state_callback, qos_profile)
		self.attitude_subscirber = self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self.attitude_callback, qos_profile)
		self.local_position_subscriber = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.local_position_callback, qos_profile)
		self.global_position_subscriber = self.create_subscription(VehicleGlobalPosition, "/fmu/out/vehicle_global_position", self.global_position_callback, qos_profile)
		self.vehicle_odometry_subscriber = self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.vehicle_odometry_callback, qos_profile)
		self.vehicle_actuator_sub = self.create_subscription(ActuatorMotors, "/fmu/out/actuator_motors", self.actuator_callback, qos_profile)
		## MAVROS
		self.imu_subscriber = self.create_subscription(Imu, "/mavros/imu/data", self.imu_callback, qos_profile_mavros)
		self.local_position_mavros_subscriber = self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.local_position_mavros_callback, qos_profile_mavros)
		self.velocity_subscriber = self.create_subscription(TwistStamped, "/mavros/local_position/velocity_body", self.velocity_callback, qos_profile_mavros)

		# Subscriber Messages Received
		## PX4
		self.vehicle_status = None
		self.state = None
		self.attitude = None
		self.local_position = None
		self.global_position = None
		self.vehicle_odometry = None
		self.actuator_thrusts = None
		## MAVROS
		self.imu_data = None
		self.local_position_mavros = None
		self.velocity = None

		# Controller
		self.dt = 0.05
		self.N = 20
		self.omega_history = []
		self.quad = Quadrotor3D()
		self.path, self.thrust_history = [], []
		self.hover = False

		# Command Loop
		self.time_period_drone = 0.05
		self.timer = self.create_timer(self.time_period_drone, self.command_loop)
		self.counter = 0

	# Subscriber Callback Functions
	## PX4
	def vehicle_status_callback(self, vehicle_status_msg):
		self.vehicle_status = vehicle_status_msg
	def state_callback(self, status_msg):
		self.status = status_msg
	def attitude_callback(self, attitude_msg):
		self.attitude = attitude_msg
	def local_position_callback(self, local_position_msg):
		self.local_position = local_position_msg
	def global_position_callback(self, global_position_msg):
		self.global_position = global_position_msg
	def vehicle_odometry_callback(self, vehicle_odometry_msg):
		self.vehicle_odometry = vehicle_odometry_msg
	## MAVROS
	def imu_callback(self, imu_msg):
		self.imu_data = imu_msg
	def local_position_mavros_callback(self, local_position_mavros_msg):
		self.local_position_mavros = local_position_mavros_msg
	def velocity_callback(self, velocity_msg):
		self.velocity = velocity_msg
	def actuator_callback(self, msg):
		self.actuator_thrusts = msg.control[:4]

	def offboard_control_heartbeat_signal_publisher(self, what_control):
		"""PX4 Offboard Mode Heartbeat Signal Publisher"""
		msg = OffboardControlMode()
		msg.position = False
		msg.velocity = False
		msg.acceleration = False
		msg.attitude = False
		msg.body_rate = False
		msg.thrust_and_torque = False
		msg.direct_actuator = False
		match what_control:
			case 'position':
				msg.position = True
			case 'velocity':
				msg.velocity = True
			case 'acceleration':
				msg.acceleration = True
			case 'attitude':
				msg.attitude = True
			case 'body_rate':
				msg.body_rate = True
			case 'thrust_and_torque':
				msg.thrust_and_torque = True
			case 'direct_actuator':
				msg.direct_actuator = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.offboard_mode_publisher.publish(msg)
	def engage_offboard_mode(self):
		"""Send Command to Engage Offboard Mode on PX4"""
		instance_num = 1
		msg = VehicleCommand()
		msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
		msg.param1 = 1.0
		msg.param2 = 6.0
		msg.param3 = 0.0
		msg.param4 = 0.0
		msg.param5 = 0.0
		msg.param6 = 0.0
		msg.param7 = 0.0
		msg.target_system = instance_num
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_command_publisher.publish(msg)
	def disengage_offboard_mode(self):
		"""Send Command to Disengage Offboard Mode on PX4"""
		instance_num = 1
		msg = VehicleCommand()
		msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
		msg.param1 = 1.0
		msg.param2 = 3.0
		msg.param3 = 0.0
		msg.param4 = 0.0
		msg.param5 = 0.0
		msg.param6 = 0.0
		msg.param7 = 0.0
		msg.target_system = instance_num
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_command_publisher.publish(msg)
	def arm(self):
		"""Send Command to PX4 for ARMING"""
		instance_num = 1
		msg = VehicleCommand()
		msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
		msg.param1 = 1.0
		msg.param2 = 0.0
		msg.param3 = 0.0
		msg.param4 = 0.0
		msg.param5 = 0.0
		msg.param6 = 0.0
		msg.param7 = 0.0
		msg.target_system = instance_num
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_command_publisher.publish(msg)
	def disarm(self):
		"""Send Command to PX4 for DISARMING"""
		instance_num = 1
		msg = VehicleCommand()
		msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
		msg.param1 = 0.0
		msg.param2 = 0.0
		msg.param3 = 0.0
		msg.param4 = 0.0
		msg.param5 = 0.0
		msg.param6 = 0.0
		msg.param7 = 0.0
		msg.target_system = instance_num
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_command_publisher.publish(msg)
	def kill(self):
		"""Send Command to PX4 for Flight Termination"""
		instance_num = 1
		msg = VehicleCommand()
		msg.command = VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION
		msg.param1 = 1.0
		msg.param2 = 0.0
		msg.param3 = 0.0
		msg.param4 = 0.0
		msg.param5 = 0.0
		msg.param6 = 0.0
		msg.param7 = 0.0
		msg.target_system = instance_num
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_command_publisher.publish(msg)
	def takeoff(self):
		"""Send Command to PX4 for TAKEOFF"""
		instance_num = 1
		msg = VehicleCommand()
		msg.command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
		msg.param1 = 0.0
		msg.param2 = 0.0
		msg.param3 = 0.0
		msg.param4 = 0.0
		msg.param5 = 0.0
		msg.param6 = 0.0
		msg.param7 = 0.0
		msg.target_system = instance_num
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_command_publisher.publish(msg)
	def land(self):
		"""Send Command to PX4 for LANDING"""
		instance_num = 1
		msg = VehicleCommand()
		msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
		msg.param1 = 0.0
		msg.param2 = 0.0
		msg.param3 = 0.0
		msg.param4 = 0.0
		msg.param5 = 0.0
		msg.param6 = 0.0
		msg.param7 = 0.0
		msg.target_system = instance_num
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_command_publisher.publish(msg)
	def set_origin(self, latitude, longitude, altitude):
		"""Send Command to PX4 for setting the ORIGIN"""
		instance_num = 1
		msg = VehicleCommand()
		msg.command = VehicleCommand.VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN
		msg.param1 = 0.0
		msg.param2 = 0.0
		msg.param3 = 0.0
		msg.param4 = 0.0
		msg.param5 = latitude
		msg.param6 = longitude
		msg.param7 = altitude
		msg.target_system = instance_num
		msg.target_component = 1
		msg.source_system = 1
		msg.source_component = 1
		msg.from_external = True
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_command_publisher.publish(msg)
	def individual_motor(self, motor_1, motor_2, motor_3, motor_4):
		"""Send Command to PX4 for Setting the Individual Thrusts to 4 Rotors"""
		msg = ActuatorMotors()
		msg.control[0] = motor_1
		msg.control[1] = motor_2
		msg.control[2] = motor_3
		msg.control[3] = motor_4
		msg.control[4] = float('nan')
		msg.control[5] = float('nan')
		msg.control[6] = float('nan')
		msg.control[7] = float('nan')
		msg.control[8] = float('nan')
		msg.control[9] = float('nan')
		msg.control[10] = float('nan')
		msg.control[11] = float('nan')
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.actuator_motors_publisher.publish(msg)
	def set_thrust(self, x, y, z):
		"""Send Command to PX4 for Setting Overall Thrust in X,Y,Z Coordinates"""
		msg = VehicleThrustSetpoint()
		msg.xyz[0] = x
		msg.xyz[1] = y
		msg.xyz[2] = z
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.vehicle_thrust_publisher.publish(msg)
	def publish_trajectory(self, x, y, z):
		"""Send Command to PX4 for Publishing Trajectory (X,Y,Z)"""
		msg = TrajectorySetpoint()
		msg.position[0] = x
		msg.position[1] = y
		msg.position[2] = z
		msg.velocity[0] = float('nan')
		msg.velocity[1] = float('nan')
		msg.velocity[2] = float('nan')
		msg.acceleration[0] = float('nan')
		msg.acceleration[1] = float('nan')
		msg.acceleration[2] = float('nan')
		msg.jerk[0] = float('nan')
		msg.jerk[1] = float('nan')
		msg.jerk[2] = float('nan')
		msg.yaw = 1.57079
		msg.yawspeed = float('nan')
		msg.timestamp = self.get_clock().now().nanoseconds//1000
		self.trajectory_publisher.publish(msg)

	def command_loop(self):
		if self.vehicle_odometry == None:
			print("No odom!!")
			return
		elif self.local_position_mavros != None and self.imu_data != None and self.velocity != None and self.counter == 0:
			position = [self.vehicle_odometry.position[0], self.vehicle_odometry.position[1], self.vehicle_odometry.position[2]]
			orientation = [self.vehicle_odometry.q[0], self.vehicle_odometry.q[1], self.vehicle_odometry.q[2], self.vehicle_odometry.q[3]]
			velocity = [self.vehicle_odometry.velocity[0], self.vehicle_odometry.velocity[1], self.vehicle_odometry.velocity[2]]
			angular_velocity = [self.vehicle_odometry.angular_velocity[0], self.vehicle_odometry.angular_velocity[1], self.vehicle_odometry.angular_velocity[2]]
			self.final_goal = [position[0], position[1], position[2]-5]

			print("starting....")
			print(orientation)
			initial_state = []
			initial_state.extend(position)
			initial_state.extend(orientation)
			initial_state.extend(velocity)
			initial_state.extend(angular_velocity)
			self.controller1 = Controller(self.quad, t_horizon=1, n_nodes=self.N, initial_state=initial_state, model_name='unbroken')
			self.controller2 = Controller(self.quad, t_horizon=1, n_nodes=self.N, initial_state=initial_state, broken=True, model_name='broken')

			self.counter += 1
			return
		elif self.counter == 0:
			return
		if self.counter < 200:
			self.individual_motor(0.0, 0.0, 0.0, 0.0)
		
		self.offboard_control_heartbeat_signal_publisher("direct_actuator")

		if self.counter == 100:
			self.engage_offboard_mode()
			self.arm()

		elif self.counter > 200:
			print("running!!!")

			position = [self.vehicle_odometry.position[0], self.vehicle_odometry.position[1], self.vehicle_odometry.position[2]]
			orientation = [self.vehicle_odometry.q[0], self.vehicle_odometry.q[1], self.vehicle_odometry.q[2], self.vehicle_odometry.q[3]]
			velocity = [self.vehicle_odometry.velocity[0], self.vehicle_odometry.velocity[1], self.vehicle_odometry.velocity[2]]
			angular_velocity = [self.vehicle_odometry.angular_velocity[0], self.vehicle_odometry.angular_velocity[1], self.vehicle_odometry.angular_velocity[2]]
			current = np.concatenate([position, orientation, velocity, angular_velocity])

			if(self.counter<400):
				thrust, x_opt = self.controller1.run_optimization(initial_state=current, goal=self.final_goal, use_model=0)
				thrust = thrust[0:4]
				print(f"pre_indi: {thrust}")
			else:
				global iter
				thrust, x_opt = self.controller2.run_optimization(initial_state=current, goal=[self.final_goal[0],self.final_goal[1],self.final_goal[2]], use_model=0)
				thrust = thrust[0:4]
				print(f"pre_indi: {thrust}")
				self.omega_history.append(np.array(angular_velocity).reshape((3,1)))
				if iter > 2:
					print(f"after_indi: {thrust}")
				iter += 1			

			thrust=((thrust*self.quad.max_thrust)/self.quad.k_1000 + 150)/1000
			self.individual_motor(thrust[0], thrust[1], thrust[2], thrust[3])

		print(self.counter)
		self.counter += 1

def main(args=None):
	rclpy.init(args=args)                            # Initial ROS2 Client
	offboard = Offboard()                            # Creating Offboard Class Object for Controller
	rclpy.spin(offboard)                             # Spin the ROS Node
	offboard.destroy_node()                          # Destroy the ROS Node
	rclpy.shutdown()                                 # Shutdown ROS2 Client

if __name__ == "__main__":
    main()                                  # Calling main function