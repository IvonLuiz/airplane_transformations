import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

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

def plot_frame(ax, transformation, label, color, scale=1):
    origin = transformation[:3, 3]
    x_axis = transformation[:3, 0]
    y_axis = transformation[:3, 1]
    z_axis = transformation[:3, 2]
    
    ax.quiver(origin[0], origin[1], origin[2], x_axis[0], x_axis[1], x_axis[2], color=color)
    ax.text(origin[0] + x_axis[0], origin[1] + x_axis[1], origin[2] + x_axis[2], 'X', color=color)
    ax.quiver(origin[0], origin[1], origin[2], y_axis[0], y_axis[1], y_axis[2], color=color)
    ax.text(origin[0] + y_axis[0], origin[1] + y_axis[1], origin[2] + y_axis[2], 'Y', color=color)
    ax.quiver(origin[0], origin[1], origin[2], z_axis[0], z_axis[1], z_axis[2], color=color)
    ax.text(origin[0] + z_axis[0], origin[1] + z_axis[1], origin[2] + z_axis[2], 'Z', color=color)


if __name__ == "__main__":
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    imu_T_cvcam = np.array([[-1., 0., 0., 0.0],
                            [0., -0.25881905, 0.96592583, 0.0],
                            [0., 0.96592583, 0.25881905, 0.0],
                            [0, 0, 0, 1]])
    drone_T_imu = np.array([
        [1., 0., 0., 0.],
        [0., 1., 0., 0.],
        [0., 0., 1., 0.3],
        [0., 0., 0., 1.]
    ])
    ned_T_enu = np.array([
        [0, 1, 0, 0],
        [1, 0, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1]
    ])
    world_T_cv = np.array([
        [1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1]
    ])

    cvcam_T_nedcam = np.array([
        [0., 1., 0., 0.],
        [0., 0., 1., 0.],
        [1., 0., 0., 0.],
        [0., 0., 0., 1.]
    ])
    drone_T_cvcam = drone_T_imu @ imu_T_cvcam
    drone_T_nedcam = drone_T_cvcam @ cvcam_T_nedcam
    
    T = drone_T_nedcam

    xyz, rpy = matrix_to_xyz_rpy(T)
    print_urdf_origin(xyz, rpy)

    # Use to visualize the transformation. 
    # Change:
    #   transformation matrix (second argument)
    #   label (third argument)
    #   color (fourth argument)
    
    plot_frame(ax, world_T_cv, 'World', 'black')
    plot_frame(ax, ned_T_enu, 'ENU', 'blue')
    plot_frame(ax, drone_T_imu, 'Drone', 'green')
    plot_frame(ax, imu_T_cvcam, 'CV', 'red')
    
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.title('Transformações de coordenadas ')
    plt.show()
    