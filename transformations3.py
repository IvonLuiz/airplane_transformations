import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# Matrizes de transformação
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

def transform_angles(roll_cam_deg, pitch_cam_deg):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    #world_frame = np.eye(4)
    #plot_frame(ax, world_frame, 'World', 'black')

    ned_cam_rot = R.from_euler('xyz', [roll_cam_deg, pitch_cam_deg, 0], degrees=True).as_matrix()
    ned_cam = np.eye(4)
    ned_cam[:3, :3] = ned_cam_rot
    plot_frame(ax, ned_cam, 'NED Camera', 'red')
    
    cv_cam = cvcam_T_nedcam @ ned_cam
    plot_frame(ax, cv_cam, 'CV Camera', 'green')
    #cam_world = ned_T_enu.T @ ned_cam
    #plot_frame(ax, cam_world, 'NED Camera', 'red')

    drone_frame = drone_T_nedcam @ ned_cam
    plot_frame(ax, drone_frame, 'Drone', 'blue')
    #drone_world = ned_T_enu.T @ drone_frame
    #print(ned_T_enu.T)
    #plot_frame(ax, drone_frame, 'Drone', 'blue')
    
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.title('Transformações de coordenadas (camera ned vermlho -> drone azul ned)')
    plt.suptitle('Em relação as coordenadas do mundo. Camera roll: {} camera pitch: {}'.format(roll_cam_deg, pitch_cam_deg),
                 fontsize=10)
    plt.show()

    r = R.from_matrix(drone_world[:3, :3])
    roll_drone_deg, pitch_drone_deg, _ = r.as_euler('xyz', degrees=True)
    
    return roll_drone_deg, pitch_drone_deg

roll_cam_deg = -0
pitch_cam_deg = -0
roll_drone_deg, pitch_drone_deg = transform_angles(roll_cam_deg, pitch_cam_deg)
print("Roll do Drone (graus):", roll_drone_deg)
print("Pitch do Drone (graus):", pitch_drone_deg)