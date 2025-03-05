import numpy as np
from scipy.spatial.transform import Rotation as R


imu_T_cvcam = np.array([[-1.,          0.,         0.,    0.0],
                        [0., -0.25881905, 0.96592583,    0.0],
                        [0.,  0.96592583, 0.25881905,    0.0],
                        [0,           0,          0,      1]])
drone_T_imu = np.array([
    [1., 0., 0., 0.],
    [0., 1., 0., 0.],
    [0., 0., 1., 0.3],
    [0., 0., 0., 1.]
])
K = [757.5139533667555, 0.0, 642.2759219526828, 0.0, 757.2011194723814, 334.5032634647471, 0.0, 0.0, 1.0]
D = [-0.34681931317666126, 0.13414619571870115, 0.0010287210615503722, -0.00014960382371076452, -0.025048620295805048]
im_size = [1280, 720]
fov_x = 80.38685625079054
fov_y = 50.856110892932804

ned_T_enu = np.array([
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, -1]
])
world_T_cv = np.array([
    [1, 0, 0],
    [0, -1, 0],
    [0, 0, -1]
])

def transform_angles(roll_cam, pitch_cam):
    # East, North, Up (ENU), used in geography
    # North, East, Down (NED), used specially in aerospace
    
    # In case of air and sea vehicles like submarines, ships, airplanes etc., which use the
    # NED-system (North-East-Down) as external reference (World frame), the vehicle's (body's)
    # positive y- or pitch axis always points to its right, and its positive z- or yaw axis always points down.
    # World frame's origin is fixed at the center of gravity of the vehicle.
    
    # NED: These axes are normally taken so that X axis is the longitudinal axis pointing ahead, Z axis is the 
    # vertical axis pointing downwards, and the Y axis is the lateral one, pointing in such a way that the frame is right-handed.
    
    # The motion of an aircraft is often described in terms of rotation about these axes, so rotation about the 
    # X-axis is called rolling, rotation about the Y-axis is called pitching, and rotation about the Z-axis is called yawing
    
    # Convert roll and pitch to a rotation vector
    cvcam_angles = np.array([roll_cam, pitch_cam, 0])
    
    # Rotate the camera frame to the IMU frame
    imu_angles = np.dot(imu_T_cvcam[:3, :3], cvcam_angles)
    
    # Rotate the IMU frame to the drone frame
    drone_angles = np.dot(drone_T_imu[:3, :3], imu_angles)
    
    # Transform from ENU to NED coordinates
    ned_angles = np.dot(ned_T_enu, drone_angles)
    
    # Transform from NED to world frame
    world_angles = np.dot(world_T_cv, ned_angles)
    
    # Extract roll and pitch from the world angles
    roll_drone = world_angles[0]
    pitch_drone = world_angles[1]
    
    return roll_drone, pitch_drone



# Script to test
if __name__ == '__main__':
    roll_drone, pitch_drone = transform_angles(-3, -30)
    print(f"Roll Drone: {roll_drone}, Pitch Drone: {pitch_drone}")