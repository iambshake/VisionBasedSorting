import torch
import cv2
import numpy as np
import pyrealsense2 as rs
import apriltag
import time

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the camera
pipeline.start(config)

# Initialize the AprilTag detector
detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))

# Create an align object
align_to = rs.stream.color
align = rs.align(align_to)

# Define init_pos correctly as a 1D array
init_pos = np.array([2.88055450e-01, 1.34160019e-18, 8.83400061e-02])

print("Waiting for 5 seconds to stabilize...")
time.sleep(5)
print("Starting frame processing...")

# Define parameters of the robotic arm
a1, a2, a3, a4 = 0, 0.25495, 0.25, 0.17415
alpha1, alpha2, alpha3, alpha4 = np.radians([90, 0, 0, 0])
d1, d2, d3, d4 = 0.11025, 0, 0, 0

# Clamp the angles to their limits
theta1_min, theta1_max = -180, 180
theta2_min, theta2_max = -5, 185
theta3_min, theta3_max = -185, 5
theta4_min, theta4_max = -95, 95
theta5_min, theta5_max = -95, 95

angles_file_path = "/home/jetson/Desktop/Abhishek/codes/joint_angles.txt"
coordinates_file_path = "/home/jetson/Desktop/Abhishek/codes/coordinates.txt"
positions_file_path = "/home/jetson/Desktop/Abhishek/codes/positions.txt"

# Define the offsets for the place positions
offset_x = 0  # Example offset in X direction
offset_y = 0  # Example offset in Y direction
offset_z = 0.25  # Example offset in Z direction

# Store positions for saving
positions = []

def arm_ik(px, py, pz):
    phi = 0

    # Handle case when px and py are both zero
    q1des = np.arctan2(py, px) if px != 0 or py != 0 else 0

    pr = np.hypot(px, py)
    z3 = pz - d1

    r2 = pr - a4 * np.cos(phi)
    z2 = z3 - a4 * np.sin(phi)

    # Check if the target point is reachable
    if r2 ** 2 + z2 ** 2 > (a2 + a3) ** 2 or r2 ** 2 + z2 ** 2 < (a2 - a3) ** 2:
        raise ValueError("Target point is not reachable")

    cos_q3 = (r2 ** 2 + z2 ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)
    cos_q3 = np.clip(cos_q3, -1.0, 1.0)

    q3des = -np.arccos(cos_q3)
    q3des_deg = np.degrees(q3des)

    if q3des_deg < theta3_min or q3des_deg > theta3_max:
        raise ValueError("Calculated q3des is out of bounds")

    sin_q2 = ((a2 + a3 * cos_q3) * z2 - a3 *
              np.sin(q3des) * r2) / (r2 ** 2 + z2 ** 2)
    cos_q2 = ((a2 + a3 * cos_q3) * r2 + a3 *
              np.sin(q3des) * z2) / (r2 ** 2 + z2 ** 2)
    q2des = np.arctan2(sin_q2, cos_q2)
    q2des_deg = np.degrees(q2des)

    if q2des_deg < theta2_min or q2des_deg > theta2_max:
        raise ValueError("Calculated q2des is out of bounds")

    q4des = phi - q2des - q3des
    q4des_deg = np.degrees(q4des)

    if q4des_deg < theta4_min or q4des_deg > theta4_max:
        raise ValueError("Calculated q4des is out of bounds")

    q1, q2, q3, q4 = map(
        np.degrees, [q1des, q2des, q3des, q4des])

    q1 = np.clip(q1, theta1_min, theta1_max)
    q2 = np.clip(q2, theta2_min, theta2_max)
    q3 = np.clip(q3, theta3_min, theta3_max)
    q4 = np.clip(q4, theta4_min, theta4_max)

    return q1, q2, q3, q4

def write_angles(angles, grip_value, file_path):
    with open(file_path, "w") as file:
        file.write(f"{angles[0]}\n")
        file.write(f"{(angles[1] - 78.463)}\n")
        file.write(f"{(angles[2] + 78.463)}\n")
        file.write(f"0\n")
        file.write(f"{angles[3]}\n")
        file.write(f"0\n")
        file.write(f'{grip_value}\n')
        file.write("1\n")
    print(
        f'Angles: {[angles[0], angles[1] - 78.463, angles[2] + 78.463, 0, angles[3], 0, grip_value]}')


def interpolate(start, end, steps):
    return np.linspace(start, end, steps)

def move_to_position(start, end, steps=2000, grip_value=50):
    waypoints = interpolate(start, end, steps)
    for point in waypoints:
        positions.append(point)  # Store the position
        try:
            angles = arm_ik(*point)
            write_angles(angles, grip_value, angles_file_path)
            time.sleep(0.00001)  # Adjust sleep time for smoother motion
        except ValueError as e:
            print(f"Error calculating IK: {e}")

def pick_and_place(object_id, goal_x, goal_y, goal_z, place_x, place_y, place_z, offset_x, offset_y, offset_z):
    print(f"Picking up object {object_id}...")
    start_pos = init_pos[:3]
    goal_pos = np.array([goal_x, goal_y, goal_z])

    # Move to the pick position with the gripper open
    move_to_position(start_pos, goal_pos, grip_value=grip_open)
    angles = arm_ik(goal_x, goal_y, goal_z)
    write_angles(angles, grip_open, angles_file_path)
    time.sleep(3)  # Wait for the arm to move to the pick position

    # Close the gripper to pick the object
    write_angles(angles, grip_closed, angles_file_path)
    time.sleep(3)  # Ensure enough time for gripper to close

    print(f"Placing object {object_id}...")
    goal_place_pos = np.array([place_x + offset_x, place_y + offset_y, place_z + offset_z])

    # Move to the place position with the gripper closed
    move_to_position(goal_pos, goal_place_pos, grip_value=grip_closed)
    angles = arm_ik(place_x + offset_x, place_y + offset_y, place_z + offset_z)
    write_angles(angles, grip_closed, angles_file_path)
    time.sleep(3)  # Wait for the arm to move to the place position

    # Open the gripper to place the object
    write_angles(angles, grip_open, angles_file_path)
    time.sleep(3)  # Ensure enough time for gripper to open

    # Move back to the initial position with the gripper open
    print("Returning to initial position...")
    move_to_position(goal_place_pos, init_pos[:3], grip_value=grip_open)
    time.sleep(3)

grip_open, grip_closed = 60, 35

# Load the YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/jetson/Desktop/Abhishek/yolov5/runs/train/sort/best.pt')

try:
    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Perform inference
        color_image = np.asanyarray(color_frame.get_data())
        results = model(color_image)
        detections = results.xyxy[0].cpu().numpy()

        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray_image)

        tag_positions = {}
        for tag in tags:
            if tag.tag_id in [1, 2]:
                cv2.polylines(color_image, [tag.corners.astype(int)], True, (0, 255, 0), 2)
                cv2.circle(color_image, tuple(int(i) for i in tag.center), 5, (0, 0, 255), -1)

                camera_z = depth_frame.get_distance(int(tag.center[0]), int(tag.center[1]))

                # Calculate the 3D coordinates
                intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
                coords_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [int(tag.center[0]), int(tag.center[1])], camera_z)

                tag_positions[tag.tag_id] = (init_pos[0] + coords_3d[2] - 0.09, -(init_pos[1] + coords_3d[0] - 0.04), init_pos[2])

                cv2.putText(color_image, f"Tag ID: {tag.tag_id}", (25, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(color_image, f"3D coordinates: X={tag_positions[tag.tag_id][0]:.4f}, Y={tag_positions[tag.tag_id][1]:.4f}, Z={tag_positions[tag.tag_id][2]:.4f}", (25, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(color_image, f'Pixel: {int(tag.center[0]), int(tag.center[1]), camera_z}', (
                    25, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        for detection in detections:
            x1, y1, x2, y2, conf, cls = detection
            if conf < 0.8:
                continue
            object_id = int(cls)
            center_x, center_y = int((x1 + x2) / 2), int((y1 + y2) / 2)
            camera_z = depth_frame.get_distance(center_x, center_y)

            # Calculate the 3D coordinates
            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            coords_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [center_x, center_y], camera_z)

            # Ignore detections that are too close (less than 15 cm)
            if camera_z > 0.15 and camera_z < 0.45:
                goal_x = init_pos[0] + coords_3d[2] - 0.12
                goal_y = -(init_pos[1] + coords_3d[0] - 0.04)
                goal_z = init_pos[2] - 0.06

                if object_id in [0, 1] and 1 in tag_positions:
                    place_x, place_y, place_z = tag_positions[1]
                elif object_id in [2, 3] and 2 in tag_positions:
                    place_x, place_y, place_z = tag_positions[2]
                else:
                    continue

                print(f'Object ID: {object_id}')
                print(f'Pixel: {center_x, center_y}')
                print(f'Position: {goal_x, goal_y, goal_z}')
                print(f'Place Position: {place_x, place_y, place_z}')

                pick_and_place(object_id, goal_x, goal_y, goal_z, place_x, place_y, place_z, offset_x, offset_y, offset_z)

        cv2.imshow('Object Detection and AprilTag Detection with Depth', color_image)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

    # Write positions to the file
    with open(positions_file_path, 'w') as file:
        for pos in positions:
            file.write(f"{pos[0]} {pos[1]} {pos[2]}\n")

    print(f"Positions saved to {positions_file_path}")
