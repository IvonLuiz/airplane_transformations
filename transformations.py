import numpy as np

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
# ENU = UTM para NED
ned_T_enu = np.array([
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, -1]
])
# Camera CV to glworld
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
    
    rotation_cam = R.from_euler('xyz', [roll_cam, pitch_cam, 0], degrees=True)
    rotation_matrix_cam = rotation_cam.as_matrix()
    print(f"Rotation cam: {rotation_matrix_cam}")
    print(f"Rotation Matrix: {rotation_matrix_cam}")

    # Rotate the camera frame to the IMU frame
    print(f"IMU_T_CVCAM: {imu_T_cvcam}")
    print(f"IMU_T_CVCAM: {imu_T_cvcam[:3, :3]}")
    rotation_matrix_imu = np.dot(imu_T_cvcam[:3, :3], rotation_matrix_cam)
    
    # Rotate the IMU frame to the drone frame
    drone_matrix = np.dot(drone_T_imu[:3, :3], rotation_matrix_imu)
    
    # Transform from ENU to NED coordinates
    drone_matrix_ned = np.dot(ned_T_enu, drone_matrix)
    
    # Transform from NED to world frame
    #drone_matrix_worl = np.dot(world_T_cv, ned_angles)
    
    # Extract roll and pitch from the world angles
    rotation_drone = R.from_matrix(drone_matrix_ned)
    rotation_drone = R.from_matrix(drone_matrix)
    roll_drone, pitch_drone, _ = rotation_drone.as_euler('xyz', degrees=True)
    
    return roll_drone, pitch_drone



# Script to test
if __name__ == '__main__':
    roll_drone, pitch_drone = transform_angles(-3, -30)
    print(f"Roll Drone: {roll_drone}, Pitch Drone: {pitch_drone}")