import subprocess
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import re
import threading
import math

# Constants (in meters)
ALEX_LENGTH = 0.28
ALEX_BREATH = 0.18
MAX_LIDAR_DISTANCE = 0.4

# Global variables
angles = np.linspace(0, 2 * np.pi, 360)
distances = np.full(angles.shape, np.inf)

# Initialize Mapping
fig = plt.figure()

# Map Alex
alex = fig.add_subplot(111, projection='polar')
alex.set_ylim(0, MAX_LIDAR_DISTANCE)  # Maximum Distance of LiDAR Data
alex.set_theta_zero_location("S")
alex_angle = math.atan(ALEX_BREATH / ALEX_LENGTH)
alex_hypotenuse = ((ALEX_LENGTH ** 2 + ALEX_BREATH ** 2) ** 0.5) / 2

# Setup radar map
ax = plt.subplot(111, projection='polar')
ax.set_ylim(0, MAX_LIDAR_DISTANCE)
ax.set_theta_zero_location("S")


def parse_data_line(line):
    """Retrieve data from ROS Terminal"""
    match = re.search(r"\[ INFO\] \[\d+\.\d+\]: : \[(-?\d+\.\d+), (\d+\.\d+|inf)\]", line)
    if match:
        angle = float(match.group(1))
        distance = float(match.group(2)) if match.group(2) != 'inf' else np.inf
        return angle, distance
    return None, None


def update_plot(frame):
    """Update Mapping"""
    ax.clear()
    plt.title('radarviz')
    valid_indices = np.isfinite(distances)
    ax.scatter(angles[valid_indices], distances[valid_indices], s=10)
    ax.set_ylim(0, MAX_LIDAR_DISTANCE)
    ax.set_theta_zero_location("S")
    alex.plot([alex_angle, np.pi - alex_angle, np.pi + alex_angle, 2 * np.pi - alex_angle, alex_angle],
              [alex_hypotenuse] * 5, 'r')


def fetch_ros_data():
    """Retrieve data from ROS Node, and update global 'distance' array"""
    cmd = "source ~/cg2111a/devel/setup.bash && " \
          "rosrun rplidar_ros rplidarNodeClient"
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True, text=True, bufsize=1, executable='/bin/bash')

    for line in iter(process.stdout.readline, ''):
        angle, distance = parse_data_line(line)
        if angle is not None and distance is not None:
            angle_rad = np.radians((angle + 180) % 360)
            index = int((angle_rad / (2 * np.pi)) * len(angles))
            distances[index] = distance


if __name__ == '__main__':
    # Read and process ROS data
    threading.Thread(target=fetch_ros_data, daemon=True).start()

    # Build animation
    ani = FuncAnimation(fig, update_plot, interval=100)
    plt.show()
