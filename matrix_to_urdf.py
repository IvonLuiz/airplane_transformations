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

def inverse_transformation(matrix):
    """
    Calculate the inverse of a 4x4 transformation matrix.
    """
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    
    # Inverse rotation
    inv_rotation = rotation.T
    
    # Inverse translation
    inv_translation = -inv_rotation @ translation
    
    # Construct the inverse transformation matrix
    inv_matrix = np.eye(4)
    inv_matrix[:3, :3] = inv_rotation
    inv_matrix[:3, 3] = inv_translation
    
    return inv_matrix

# Example usage:
if __name__ == "__main__":
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Identity reference frame used for visualization on plotting.
    ref_frame = np.identity(4)
    
    # Example transformation matrices (change this to your actual matrices)
    target_T_source = np.array([
        [ 0., -1.,  0.,  0.5],
        [ 0.,  0.,  1.,  0.5],
        [-1., -0.,  0.,  0.5],
        [ 0.,  0.,  0.,  1.]
    ])
    # Print like this:
    print("SOURCE TO TARGET")
    xyz, rpy = matrix_to_xyz_rpy(target_T_source)
    print_urdf_origin(xyz, rpy)
    
    # If you want to see the inverse transformation, uncomment the following line:
    # source_T_target = inverse_transformation(target_T_source)
    # print("TARGET TO SOURCE")
    # xyz, rpy = matrix_to_xyz_rpy(source_T_target)
    # print_urdf_origin(xyz, rpy)
    
    # You can visualize multiple transformations in relation to the 'ref_frame'.
    # Use to visualize the transformation.
    
    # Plot all frames with RGB axes and labels
    plot_frame(ax, ref_frame, "ref", scale=4)
    plot_frame(ax, target_T_source, "target_T_source", scale=4)
    # plot_frame(ax, other_transformation, "other transformation", scale=4)
    
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()
    