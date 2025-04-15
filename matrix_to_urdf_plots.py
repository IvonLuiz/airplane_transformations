import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from math import radians, cos, sin
def matrix_to_xyz_rpy(matrix):
    """
    Convert a 4x4 transformation matrix to URDF-friendly XYZ and RPY.
    """
    xyz = matrix[:3, 3]
    rot_matrix = matrix[:3, :3]

    # Convert to roll-pitch-yaw (ZYX order)
    r = R.from_matrix(rot_matrix)
    rpy = r.as_euler('xyz', degrees=False)

    return xyz, rpy

def print_urdf_origin(xyz, rpy, precision=12):
    """
    Print the URDF origin tag with xyz and rpy.
    """
    np.set_printoptions(suppress=True, precision=precision)
    
    xyz_str = " ".join([f"{v}" for v in xyz])
    rpy_str = " ".join([f"{v}" for v in rpy])
    print(f'<origin xyz="{xyz_str}" rpy="{rpy_str}" />')

def plot_frame(ax, transformation, label, scale=1):
    """
    Plot a coordinate frame with RGB-colored axes (X=Red, Y=Green, Z=Blue).
    Also adds a label for the frame.
    """
    origin = transformation[:3, 3]
    x_axis = transformation[:3, 0]
    y_axis = transformation[:3, 1]
    z_axis = transformation[:3, 2]
    
    # Plot X-axis (Red)
    ax.quiver(origin[0], origin[1], origin[2], x_axis[0], x_axis[1], x_axis[2], color='red', length=0.2*scale)
    ax.text(origin[0] + x_axis[0]*0.2, origin[1] + x_axis[1]*0.2, origin[2] + x_axis[2]*0.2, 'X', color='red')
    
    # Plot Y-axis (Green)
    ax.quiver(origin[0], origin[1], origin[2], y_axis[0], y_axis[1], y_axis[2], color='green', length=0.2*scale)
    ax.text(origin[0] + y_axis[0]*0.2, origin[1] + y_axis[1]*0.2, origin[2] + y_axis[2]*0.2, 'Y', color='green')
    
    # Plot Z-axis (Blue)
    ax.quiver(origin[0], origin[1], origin[2], z_axis[0], z_axis[1], z_axis[2], color='blue', length=0.2*scale)
    ax.text(origin[0] + z_axis[0]*0.2, origin[1] + z_axis[1]*0.2, origin[2] + z_axis[2]*0.2, 'Z', color='blue')
    
    # Add frame label at origin
    ax.text(origin[0], origin[1], origin[2], label, fontsize=10, ha='center', va='center')

if __name__ == "__main__":
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ref_nedcam = np.array([
        [1, 0, 0, 0.1],
        [0, 1, 0, 0.1],
        [0, 0, 1, 0.1],
        [0, 0, 0, 1]
    ])

    #ins_T_cvcam = np.array([[-1., 0., 0., 0.0],
    #                        [0., -0.25881905, 0.96592583, 0.0],
    #                        [0., 0.96592583, 0.25881905, 0.0],
    #                        [0, 0, 0, 1]])

    #imu_T_cvcam = np.array([[-1., 0., 0., 0.0],
    #                        [0., -0.25881905, 0.96592583, 0.0],
    #                        [0., 0.96592583, 0.25881905, 0.0],
    #                        [0, 0, 0, 1]])

    ins_T_cvcam = np.array([
        [0, 1, 0, 0],
        [-1, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    drone_T_ins = np.array([
        [0., 1., 0., 0.],
        [0., 0., 1., 0.],
        [1., 0., 0., 0.],
        [0., 0., 0., 1.]
    ])
    xyz, rpy = matrix_to_xyz_rpy(drone_T_ins)
    print_urdf_origin(xyz, rpy)

    cvcam_T_nedcam = np.array([
        [0., 1., 0., 0.],
        [0., 0., 1., 0.],
        [1., 0., 0., 0.],
        [0., 0., 0., 1.]
    ])

    drone_T_cvcam = drone_T_ins @ ins_T_cvcam
    drone_T_nedcam = drone_T_cvcam @ cvcam_T_nedcam
    
    #drone_T_cvcam = np.array([
    #    [0., 0., 1., 0.],
    #    [1., 0., 0., 0.],
    #    [0., 1., 0., 1.],
    #    [0., 0., 0., 1.]
    #])
    drone_T_nedcam = drone_T_cvcam @ cvcam_T_nedcam
    #rotate 15 degrees upwards around Y axis
    angle_deg = -15
    rot_matrix = R.from_euler('y', angle_deg, degrees=True).as_matrix()
    rot_T = np.eye(4)
    rot_T[:3, :3] = rot_matrix 
    drone_T_nedcam = drone_T_nedcam @ rot_T
    
    
    T = drone_T_nedcam
    print("drone_T_nedcam")
    print(T)

    xyz, rpy = matrix_to_xyz_rpy(T)
    print_urdf_origin(xyz, rpy)

    # Use to visualize the transformation. 
    # Change:
    #   transformation matrix (second argument)
    #   label (third argument)
    #   color (fourth argument)
    
    # Plot all frames with RGB axes and labels
    plot_frame(ax, ref_nedcam, "nedcam", scale=4)  # World frame (identity)
    plot_frame(ax, cvcam_T_nedcam, "cvcam", scale=4)
    plot_frame(ax, ins_T_cvcam, "ins", scale=4)
    plot_frame(ax, drone_T_cvcam, "Drone", scale=4)
    
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.title('Transformações de coordenadas ')
    plt.show()
    