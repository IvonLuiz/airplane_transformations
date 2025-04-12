from scipy.spatial.transform import Rotation as R
import numpy as np
import math


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

cvcam_T_nedcam = np.array([
    [0., 1., 0., 0.],
    [0., 0., 1., 0.],
    [1., 0., 0., 0.],
    [0., 0., 0., 1.]
])

drone_T_cvcam = drone_T_imu @ imu_T_cvcam
drone_T_nedcam = drone_T_cvcam @ cvcam_T_nedcam

def extract_xyz_rpy(T):
    rotation_matrix = T[:3, :3]
    
    # Extract the elements of the rotation matrix
    r11, r12, r13 = rotation_matrix[0]
    r21, r22, r23 = rotation_matrix[1]
    r31, r32, r33 = rotation_matrix[2]

    # Compute angles using trigonometry
    # yaw, pitch, and roll
    yaw = math.atan2(r21, r11)
    pitch = math.atan2(-r31, math.sqrt(r32**2 + r33**2))
    roll = math.atan2(r32, r33)

    # Convert to degrees for readability
    yaw_deg = math.degrees(yaw)
    pitch_deg = math.degrees(pitch)
    roll_deg = math.degrees(roll)

    translation_vector = T[:3, 3]

    print("Results with trigonometry:")
    print(f"Roll (φ): {roll_deg:.2f}°")
    print(f"Pitch (θ): {pitch_deg:.2f}°")
    print(f"Yaw (ψ): {yaw_deg:.2f}°")
    print(f"Translation: {translation_vector}")
    
    # Compute angles using scipy
    urdf_rpy = R.from_matrix(rotation_matrix).as_euler('xyz')
    urdf_translation = T[:3,3]

    # Convert to degrees for readability
    urdf_rpy_deg = np.degrees(urdf_rpy)
    print("Results with scipy:")
    print(f"Roll (φ): {urdf_rpy_deg[0]:.2f}°")
    print(f"Pitch (θ): {urdf_rpy_deg[1]:.2f}°")
    print(f"Yaw (ψ): {urdf_rpy_deg[2]:.2f}°")
    print(f"Translation: {urdf_translation}")

    return translation_vector, roll_deg, pitch_deg, yaw_deg, urdf_rpy, urdf_translation

if __name__ == "__main__":
    # Get roll pitch yaw from transformations matrixes
    translation_vector, roll_deg, pitch_deg, yaw_deg, rpy, translation= extract_xyz_rpy(drone_T_nedcam)
    np.set_printoptions(suppress=True, precision=12)
    print(translation)
    print(np.round(rpy, 12))

    #print(f"Translation: {translation_vector}")
    #print(f"Roll: {roll_deg}")
    #print(f"Pitch: {pitch_deg}")
    #print(f"Yaw: {yaw_deg}")
    ## print rpy with 12 decimal places
    #print(f"Rotation: {rpy[0]:.12f}, {rpy[1]:.12f}, {rpy[2]:.12f}")
    #print(f"Translation: {translation}")
