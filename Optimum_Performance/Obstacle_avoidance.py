import numpy as np
import matplotlib.pyplot as plt
import os
import sys
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import Data, ISA, MissionType, plt


def energy_height(V: float, h: float) -> float:
    """
    Calculate the energy height at a given altitude.
    """
    return h + V**2 / (2 * 9.81)  # h in meters, V in m/s, energy height in meters

def pull_up_radius(n: float, V: float) -> float:
    """
    Calculate the turn radius for a given load factor and velocity.
    """
    return V**2 / (9.81 * (n-1))  # n is the load factor, V in m/s, turn radius in meters

def turn_radius(n: float, V: float) -> float:
    """
    Calculate the turn radius for a given load factor and velocity.
    """
    return V**2 / (9.81 * np.sqrt(n**2-1))  # n is the load factor, V in m/s, turn radius in meters

def plot_pullup_trajectory(aircraft_data, reaction_time=1.0):
    max_n_pullup = aircraft_data.data['outputs']['general']['nmax']
    v_start = aircraft_data.data['requirements']['cruise_speed']
    v_end = aircraft_data.data['requirements']['stall_speed_clean']

    # Example obstacles: list of (distance, height)
    obstacles = [
        (150, 3),
        (280, 20),
        (350, 40)
    ]

    # Initial conditions
    dt = 0.1  # time step in seconds
    x_traj = [0]
    y_traj = [0]
    theta = 0  # initial flight path angle (radians)
    V = v_start
    h = 0
    energy_h = energy_height(v_start, h)  # Initial energy height

    # Add straight segment for pilot reaction time
    straight_distance = reaction_time * V
    num_steps = int(straight_distance // (V * dt))
    for _ in range(num_steps):
        dx = V * dt
        dy = 0
        x_traj.append(x_traj[-1] + dx)
        y_traj.append(y_traj[-1] + dy)

    # Stop when height exceeds a target (e.g., clear the highest obstacle + margin)
    target_height = max([h for _, h in obstacles]) + 10  # 10 m margin above highest obstacle
    while y_traj[-1] < target_height:
        # At each step, calculate turn radius for max load factor
        n = max_n_pullup
        R = pull_up_radius(n, V)
        # Change in flight path angle
        dtheta = (V / R) * dt
        theta += dtheta
        # Distance increments
        dx = V * np.cos(theta) * dt
        dy = V * np.sin(theta) * dt
        x_traj.append(x_traj[-1] + dx)
        y_traj.append(y_traj[-1] + dy)
        h = y_traj[-1]
        # Update V to keep energy height constant: h + V^2/(2g) = energy_h
        # V_new_sq = max(0, (energy_h - h) * 2 * 9.81)
        # V = np.sqrt(V_new_sq) if V_new_sq > 0 else v_end
        # if V <= v_end:
        #     break

    # Plot the trajectory
    plt.figure(figsize=(10, 6))
    plt.plot(x_traj, y_traj, label="Pull-up Trajectory")

    # Plot obstacles as vertical lines and annotate them
    for idx, (dist, height) in enumerate(obstacles):
        plt.vlines(dist, 0, height, colors='r', linestyles='dashed', label='Obstacle' if idx == 0 else None)
        plt.scatter(dist, height, color='r')
        plt.annotate(f"Obstacle {idx+1}\n({dist} m away, {height} m high)", (dist, height),
                     textcoords="offset points", xytext=(5, 10), ha='left', color='r', fontsize=9)

    plt.xlabel("Distance to Obstacle [m]")
    plt.ylabel("Height [m]")
    plt.title("Aircraft Pull-up Trajectory with Obstacles")
    plt.grid(True)
    plt.legend()
    plt.show()

def plot_turn_trajectory(aircraft_data, reaction_time=1.0):
    max_n_turn = aircraft_data.data['outputs']['general']['max_n_turn']
    v_start = aircraft_data.data['requirements']['cruise_speed']
    v_end = aircraft_data.data['requirements']['stall_speed_clean']

    # Example obstacles: list of (x, y) positions in meters (from above)
    # Make obstacles symmetric about x=0
    base_obstacles = [
        (4, 300),
        (10, 360),
        (30, 450)
    ]
    obstacles = []
    for x, y in base_obstacles:
        obstacles.append((x, y))
        obstacles.append((-x, y))

    # Initial conditions: aircraft starts at (0, 0), heading straight "up" (positive y)
    dt = 0.1  # time step in seconds
    x_traj = [0]
    y_traj = [0]
    theta = np.pi / 2  # initial heading angle: up (90 degrees)
    V = v_start

    # Add straight segment for pilot reaction time
    straight_distance = reaction_time * V
    num_steps = int(straight_distance // (V * dt))
    for _ in range(num_steps):
        dx = V * np.cos(theta) * dt
        dy = V * np.sin(theta) * dt
        x_traj.append(x_traj[-1] + dx)
        y_traj.append(y_traj[-1] + dy)

    # Stop when y exceeds the last obstacle's y + margin
    target_y = max([y for _, y in obstacles]) + 50  # 50 m margin after last obstacle

    # Main (right) turn trajectory
    x_traj_right = x_traj.copy()
    y_traj_right = y_traj.copy()
    theta_right = theta
    while y_traj_right[-1] < target_y:
        n = max_n_turn
        R = turn_radius(n, V)
        dtheta = (V / R) * dt
        theta_right += dtheta
        dx = V * np.cos(theta_right) * dt
        dy = V * np.sin(theta_right) * dt
        x_traj_right.append(x_traj_right[-1] + dx)
        y_traj_right.append(y_traj_right[-1] + dy)

    # Mirrored (left) turn trajectory
    x_traj_left = [ -x for x in x_traj ]
    y_traj_left = y_traj.copy()
    theta_left = theta
    while y_traj_left[-1] < target_y:
        n = max_n_turn
        R = turn_radius(n, V)
        dtheta = -(V / R) * dt  # Negative for left turn
        theta_left += dtheta
        dx = V * np.cos(theta_left) * dt
        dy = V * np.sin(theta_left) * dt
        x_traj_left.append(x_traj_left[-1] + dx)
        y_traj_left.append(y_traj_left[-1] + dy)

    plt.figure(figsize=(10, 6))
    plt.plot(x_traj_right, y_traj_right, label="Turn Trajectory (Right)", color='b')
    plt.plot(x_traj_left, y_traj_left, label="Turn Trajectory (Left)", color='b', linestyle='dotted')

    # Plot obstacles and join symmetric pairs with a dotted line, annotate them
    for idx, (x, y) in enumerate(base_obstacles):
        plt.scatter([x, -x], [y, y], color='r', label='Obstacle' if idx == 0 else None)
        plt.plot([x, -x], [y, y], 'r:', linewidth=1)
        plt.annotate(f"Obs {idx+1}\n({x*2} m wide, {y} m away)", (x, y), textcoords="offset points", xytext=(20, 10), ha='left', color='r', fontsize=9)

    plt.xlabel("X Position [m]")
    plt.ylabel("Y Position [m]")
    plt.title("Aircraft Turn Trajectory (Top View)")
    plt.grid(True)
    plt.legend()
    plt.axis('equal')
    plt.show()

if __name__ == "__main__":
    file_path = "design3.json"
    aircraft_data = Data(file_path)
    reaction_time = 1.  # seconds

    plot_pullup_trajectory(aircraft_data, reaction_time=reaction_time)
    plot_turn_trajectory(aircraft_data, reaction_time=reaction_time)