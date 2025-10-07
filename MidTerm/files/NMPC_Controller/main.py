import numpy as np
import argparse
import timeit
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import time
from mpl_toolkits.mplot3d import Axes3D 

from quadrotor import Quadrotor3D
from controller import Controller

# Print statement formatting
def print_quad_state(iteration = '', current=None, thrust=None, hover_status=None):
    hover_line = f"Hover Status:                 {('True' if hover_status else 'False')}" if hover_status is not None else ""
    print(f"""
    ------------------------ Quad State Iteration {iteration} ------------------------
    Position (x, y, z):           ({current[0]:.4f}, {current[1]:.4f}, {current[2]:.4f})
    Quaternion (q0, q1, q2, q3):  ({current[3]:.4f}, {current[4]:.4f}, {current[5]:.4f}, {current[6]:.4f})
    Velocity (vx, vy, vz):        ({current[7]:.4f}, {current[8]:.4f}, {current[9]:.4f})
    Angular Rate (wx, wy, wz):    ({current[10]:.4f}, {current[11]:.4f}, {current[12]:.4f})
    Thrust (t0, t1, t2, t3):      ({thrust[0]:.4f}, {thrust[1]:.4f}, {thrust[2]:.4f}, {thrust[3]:.4f})
    {hover_line}
    --------------------------------------------------------------------------
    """)


def ControlledLandingTrajectory(sim_time, dt):

    # Controlled landing trajectory that follows a converging helix
    vertical_speed = -0.5
    initial_radius = 3
    convergence_rate = 0.15
    initial_z = 10

    xref, yref, zref = [], [], []

    for i in range(int(sim_time / dt)):
        t = dt * i
        radius = initial_radius * np.exp(-convergence_rate * t)
        x = radius * np.sin(t)
        y = radius * (1- np.cos(t))
        z = max(initial_z + vertical_speed * t, 0.2)  # Ensure z doesn't go below 0.2

        xref.append(x)
        yref.append(y)
        zref.append(z)
    
    return np.array(xref), np.array(yref), np.array(zref)

def ControlledLanding():
    dt = 0.01
    sim_time = 24   
    N = 200
    quad = Quadrotor3D()
    controller = Controller(quad, t_horizon=2*N*dt, n_nodes=N)

    xref, yref, zref = ControlledLandingTrajectory(sim_time, dt)
    path, time_record, velocity_record = [], [], []

    # Declare initial state for controlled landing
    quad.pos = np.array([0, 0, 10])
    quad.vel = np.array([1, 3, 2])
    quad.angle = np.array([1., 0., 0., 0.])
    quad.a_rate = np.array([1, 2, 2])
    thrust = np.array([2, 2, 3, 4])
    current = np.concatenate([quad.pos, quad.angle, quad.vel, quad.a_rate])
    print("This is the current state from which we are trying to land")
    print_quad_state(None, current, thrust)
    time.sleep(2)

    # Controlled descent loop
    for i in range(int(sim_time/dt)):
        x, y, z = xref[i:i+N+1], yref[i:i+N+1], zref[i:i+N+1]
        if len(x) < N+1:
            x = np.pad(x, (0, N+1-len(x)), constant_values=xref[-1])
            y = np.pad(y, (0, N+1-len(y)), constant_values=yref[-1])
            z = np.pad(z, (0, N+1-len(z)), constant_values=zref[-1])
        goal = np.array([x, y, z]).T

        current = np.concatenate([quad.pos, quad.angle, quad.vel, quad.a_rate])
        start = timeit.default_timer()
        thrust = controller.run_optimization(initial_state=current, goal=goal, mode='traj', use_model=0)[:4]
        time_record.append(timeit.default_timer() - start)
        print_quad_state(i, current, thrust)
        quad.update(thrust, dt)
        path.append(quad.pos)
        velocity_record.append(quad.vel)

    # Timing info
    print("Average estimation time: {:.5f}".format(np.mean(time_record)))
    print("Max estimation time: {:.5f}".format(np.max(time_record)))
    print("Min estimation time: {:.5f}".format(np.min(time_record)))

    # Visualization
    path = np.array(path)
    velocity_record = np.array(velocity_record)
    time_axis = np.linspace(0, sim_time, len(velocity_record))

    # 3D path plot
    plt.figure()
    ax = plt.axes(projection='3d')
    #ax.plot(xref, yref, zref, c=[1,0,0], label='goal')
    ax.plot(path[:,0], path[:,1], path[:,2], label='trajectory')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.legend()

    # CPU time plot
    plt.figure()
    plt.plot(time_record)
    plt.ylabel('CPU Time [s]')
    plt.xlabel('Iteration')
    plt.title('Computation Time per Iteration')

    # Velocity plots
    plt.figure()
    plt.plot(time_axis, velocity_record[:, 0], label='x-velocity')
    plt.plot(time_axis, velocity_record[:, 1], label='y-velocity')
    plt.plot(time_axis, velocity_record[:, 2], label='z-velocity')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.title('Velocity vs Time')
    plt.legend()

    plt.show()

def move2Goal():
    dt = 0.01
    N = 100
    quad = Quadrotor3D()
    controller = Controller(quad, t_horizon=2 * N * dt, n_nodes=N)
    final_goal = np.array([4,6,3])

    # Determine the number of waypoints and set them up
    num_waypoints = max(int(max(final_goal) / 10) + 1, 1)
    waypoints = [quad.pos + i * (final_goal - quad.pos) / num_waypoints for i in range(1, num_waypoints + 1)]

    path, thrust_history = [], []
    iteration = 0
    vicinity_radius = 0.2

    # Navigation loop through waypoints
    for i, goal in enumerate(waypoints):
        # Set a smaller radius for the final waypoint to ensure precise arrival
        if i == len(waypoints) - 1:
            vicinity_radius = 10e-4

        while np.linalg.norm(goal - quad.pos) > vicinity_radius:
            current = np.concatenate([quad.pos, quad.angle, quad.vel, quad.a_rate])
            thrust = controller.run_optimization(initial_state=current, goal=goal, use_model=0)[:4]
            thrust_history.append(thrust)
            quad.update(thrust, dt)
            path.append(quad.pos)
            print_quad_state(iteration, current, thrust)
            iteration += 1

    print("Goal tracked successfully!")



    # Visualization
    path = np.array(path)
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot(path[:, 0], path[:, 1], path[:, 2], label='path')
    ax.scatter(final_goal[0], final_goal[1], final_goal[2], c='red', label='final goal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.legend()
    plt.show()


def StableHover():
    dt, N = 0.01, 100
    quad = Quadrotor3D()
    controller = Controller(quad, t_horizon=2 * N * dt, n_nodes=N)
    final_goal = np.array([5, 3, 6])

    path, thrust_history, counter = [], [], 0
    iteration = 0  # Add iteration counter
    hover = False
    # Hover at goal until stability is reached
    while counter < 100:
        if np.linalg.norm(final_goal - quad.pos) < 0.01:
            hover = True
            counter += 1
            if counter == 1:
                print("Drone is now hovering!")
                time.sleep(2)
        current = np.concatenate([quad.pos, quad.angle, quad.vel, quad.a_rate])
        thrust = controller.run_optimization(initial_state=current, goal=final_goal, use_model=0)[:4]
        thrust_history.append(thrust)
        quad.update(thrust, dt)
        path.append(quad.pos)
        print_quad_state(iteration, current, thrust, hover)  # Use iteration counter
        iteration += 1  # Increment counter

    # Visualization
    path = np.array(path)
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot(path[:, 0], path[:, 1], path[:, 2], label='path')
    ax.scatter(final_goal[0], final_goal[1], final_goal[2], c='red', label='final goal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.legend()
    plt.show()


def BasicNavigationTrajectory(sim_time, dt):
    # Generate 8-shape trajectory
    xref, yref, zref = [], [], []

    for i in range(int(sim_time / dt)):
        t = dt * i
        radius = 8 * np.exp(-0.07 * t)
        x = radius * np.sin(2 * t)
        y = radius * np.sin(t)
        z = 3

        xref.append(x)
        yref.append(y)
        zref.append(z)

    return np.array(xref), np.array(yref), np.array(zref)

def BasicNavigation():
    dt, sim_time, N = 0.01, 15, 200
    quad = Quadrotor3D()
    controller = Controller(quad, t_horizon=2*N*dt, n_nodes=N)

    xref, yref, zref = BasicNavigationTrajectory(sim_time, dt)
    path, time_record = [], []

    # Navigation loop following 8-shape trajectory
    for i in range(int(sim_time/dt)):
        x, y, z = xref[i:i+N+1], yref[i:i+N+1], zref[i:i+N+1]
        if len(x) < N+1:
            x = np.pad(x, (0, N+1-len(x)), constant_values=xref[-1])
            y = np.pad(y, (0, N+1-len(y)), constant_values=yref[-1])
            z = np.pad(z, (0, N+1-len(z)), constant_values=zref[-1])
        goal = np.array([x, y, z]).T

        current = np.concatenate([quad.pos, quad.angle, quad.vel, quad.a_rate])
        start = timeit.default_timer()
        thrust = controller.run_optimization(initial_state=current, goal=goal, mode='traj', use_model=0)[:4]
        time_record.append(timeit.default_timer() - start)
        print_quad_state(i, current, thrust)
        quad.update(thrust, dt)
        path.append(quad.pos)

    # Timing info
    print("Average estimation time: {:.5f}".format(np.mean(time_record)))
    print("Max estimation time: {:.5f}".format(np.max(time_record)))
    print("Min estimation time: {:.5f}".format(np.min(time_record)))

    # Visualization
    path = np.array(path)
    plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot(xref, yref, zref, c=[1,0,0], label='goal')
    ax.plot(path[:,0], path[:,1], path[:,2])
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.legend()

    plt.figure()
    plt.plot(time_record)
    plt.ylabel('CPU Time [s]')
    plt.show()

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Select a function to execute and optional goal coordinates.")
    parser.add_argument("function", choices=["move2Goal", "StableHover", "BasicNavigation", "ControlledLanding"],
                        help="The function to execute")

    args = parser.parse_args()

    # Call the function based on the command-line argument
    if args.function == "move2Goal":
        move2Goal()
    elif args.function == "StableHover":
        StableHover()
    elif args.function == "BasicNavigation":
        BasicNavigation()
    elif args.function == "ControlledLanding":
        ControlledLanding()