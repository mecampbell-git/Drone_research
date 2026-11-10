
import numpy as np
from scipy.spatial.transform import Rotation as R

# TODO (real camera): pixel_to_ground_ned() models an ideal pinhole centred on the image,
# with fx, fy from fov_rad. That fits the sim camera only. The real See3CAM_24CUG has a
# fisheye lens, and no fov_rad works for it: the ray angle is off by ~12 deg at the left
# and right edges and ~20 deg in the corners (and ~1 deg at the centre, since the lens's
# centre is at (939, 595), not (960, 600)). Changes needed:
#   - Replace fov_rad and img_size with K (3x3) and D (4 coefficients) loaded from
#     camera/calibration/see3cam_24cug_<W>x<H>_equidistant.yaml (ROS camera_info format),
#     with <W>x<H> from the mission JSON's camera.img_size. Calibrated modes: 1920x1200,
#     1920x1080, 1280x720. The YAMLs aren't installed with px4_offboard yet; add them to
#     setup.py's data_files or pass a path.
#   - Step 1 becomes:
#         n = cv2.fisheye.undistortPoints(np.array([[[u, v]]], float), K, D).ravel()
#         v_c = np.array([n[0], n[1], 1.0])
#   - Sim: the Gazebo camera is a pinhole, K = [[1042.2, 0, 959.5], [0, 1042.2, 599.5], [0, 0, 1]]
#     with no distortion. Use cv2.undistortPoints(..., K, None) for it (or keep the FOV path
#     with fov (85.3, 59.9)); /camera/camera_info also carries this K.
#   - Feed it raw detector pixels (the boxes on /detections are in the distorted image);
#     don't undistort the image first.
#   - Recalibrate (tools/calibrate_camera.py) whenever the lens focus is changed.

def pixel_to_ground_ned(
    u, v,
    drone_pos_ned,      # [North, East, Down] (Down is negative, e.g., -50.0)
    drone_rpy_rad,      # [roll, pitch, yaw] in radians
    mount_rpy_rad,      # [roll, pitch, yaw] relative to body in radians
    fov_rad,            # (fov_h, fov_v) in radians
    img_size            # (width, height) in pixels
):
    W, H = img_size
    fov_h, fov_v = fov_rad
    
    # 1. Camera intrinsics
    fx = W / (2.0 * np.tan(fov_h / 2.0))
    fy = H / (2.0 * np.tan(fov_v / 2.0))
    cx, cy = W / 2.0, H / 2.0
    
    # Ray in optical frame (x right, y down, z forward)
    v_c = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    
    # 2. Camera optical -> Body FRD
    # x_b = z_c (forward), y_b = x_c (right), z_b = y_c (down)
    R_frd_opt = np.array([
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0]
    ])
    
    # Mount rotation applied in body frame (aerospace sequence Z-Y-X / yaw-pitch-roll)
    R_mount = R.from_euler('ZYX', [mount_rpy_rad[2], mount_rpy_rad[1], mount_rpy_rad[0]]).as_matrix()
    R_b_c = R_mount @ R_frd_opt
    
    # 3. Body FRD -> World NED (standard aerospace: yaw -> pitch -> roll)
    roll, pitch, yaw = drone_rpy_rad
    R_w_b = R.from_euler('ZYX', [yaw, pitch, roll]).as_matrix()
    
    # Ray in NED frame
    d_w = R_w_b @ R_b_c @ v_c
    
    # 4. Ray-plane intersection at Down = 0
    N0, E0, D0 = drone_pos_ned
    
    # In NED, pointing down means d_w[2] > 0
    if d_w[2] <= 0:
        raise ValueError("Ray does not point downward; no ground intersection.")
        
    s = -D0 / d_w[2]
    
    n_ground = N0 + s * d_w[0]
    e_ground = E0 + s * d_w[1]
    
    return np.array([n_ground, e_ground, 0.0])


def target_waypoint(position, target, height, mount_angle):
    """
    Calculate a waypoint for the drone to fly to, given a target position on the ground.
    
    :param position: Current position of the drone in NED coordinates (numpy array).
    :param target: Target position on the ground in NED coordinates (numpy array).
    :param height: Desired height above the ground for the waypoint.
    :param mount_angle: Mount angle of the camera in radians.
    :return: Waypoint position in NED coordinates (numpy array).
    """
    # Calculate the direction vector from the drone to the target
    direction = target - position
    direction[2] = 0  # Ignore vertical component for horizontal distance
    
    # Normalize the direction vector
    norm = np.linalg.norm(direction)
    
    direction /= norm

    dxy = height/np.tan(mount_angle)

    # Calculate the waypoint position
    waypoint = direction * (norm - dxy) + position
    waypoint[2] = -height  # Set the desired height above the ground 
    return waypoint
