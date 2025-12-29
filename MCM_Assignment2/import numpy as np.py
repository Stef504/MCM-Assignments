import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- 1. Math Helper Functions ---
def rot_z(psi):
    psi = np.radians(psi)
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def rot_y(theta):
    theta = np.radians(theta)
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rot_x(phi):
    phi = np.radians(phi)
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def make_transform(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T

# --- 2. Calculate Transformations (From previous step) ---
# T_01 (Base to 1)
T_01 = np.array([
    [1, 0, 0, 0.0],
    [0, 1, 0, 0.0],
    [0, 0, 1, 0.105],
    [0, 0, 0, 1.0]
])

# T_12
pose_12 = np.array([0, 0, 0.110])
rotation_12 = rot_z(-90) @ rot_y(-90)
T_12 = make_transform(rotation_12, pose_12)

# T_23
pose_23 = np.array([0.1, 0, 0])
rotation_23 = rot_y(90) @ rot_z(180)
T_23 = make_transform(rotation_23, pose_23)

# T_34
pose_34 = np.array([0, 0, 0.325])
rotation_34 = rot_y(90) @ rot_z(180) 
T_34 = make_transform(rotation_34, pose_34)

# T_45
pose_45 = np.array([0.095, 0, 0])
rotation_45 = rot_y(90) @rot_z(-90)
T_45 = make_transform(rotation_45, pose_45)

# T_56
pose_56 = np.array([0, 0, 0.095])
rotation_56 = rot_z(-180) 
T_56 = make_transform(rotation_56, pose_56)

# T_67
pose_67 = np.array([0, 0, 0.345])
rotation_67 = np.eye(3)
T_67 = make_transform(rotation_67, pose_67)

# T7E
pose_7EE = np.array([0, 0, 0.060])
rotation_7EE = np.eye(3)
T_7EE = make_transform(rotation_7EE, pose_7EE)

# --- 3. Compute Global Transformations (Base to Each Frame) ---
# To plot them, we need every frame with respect to the Base (Frame 0)
transforms_local = [T_01, T_12, T_23, T_34, T_45, T_56, T_67, T_7EE]
transforms_global = []

current_T = np.eye(4)
transforms_global.append(current_T) # Add Base Frame (0,0,0)

for T in transforms_local:
    current_T = current_T @ T
    transforms_global.append(current_T)

# --- 4. Visualization Logic ---

def set_axes_equal(ax):
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc. This is crucial for visualizing rotations correctly.
    """
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Axis length for visualization
L = 0.5 

for i, T in enumerate(transforms_global):
    # Extract Position (Translation)
    origin = T[:3, 3]
    
    # Extract Orientation (Rotation Columns)
    # Column 0 = X axis direction
    # Column 1 = Y axis direction
    # Column 2 = Z axis direction
    x_axis = T[:3, 0]
    y_axis = T[:3, 1]
    z_axis = T[:3, 2]
    
    # Plot X axis (Red)
    ax.quiver(origin[0], origin[1], origin[2], 
              x_axis[0], x_axis[1], x_axis[2], 
              length=L, color='r', normalize=True)
    
    # Plot Y axis (Green)
    ax.quiver(origin[0], origin[1], origin[2], 
              y_axis[0], y_axis[1], y_axis[2], 
              length=L, color='g', normalize=True)
    
    # Plot Z axis (Blue)
    ax.quiver(origin[0], origin[1], origin[2], 
              z_axis[0], z_axis[1], z_axis[2], 
              length=L, color='b', normalize=True)
    
    # Label the frame origin
    label = f"{{{i}}}" if i > 0 else "{Base}"
    ax.text(origin[0], origin[1], origin[2], label, fontsize=12, fontweight='bold')

# Plot lines connecting the origins to visualize the robot links
origins = np.array([T[:3, 3] for T in transforms_global])
ax.plot(origins[:, 0], origins[:, 1], origins[:, 2], 'k--', alpha=0.5, label='Links')

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Robot Coordinate Frames Visualization')
ax.legend()

set_axes_equal(ax)
plt.show()