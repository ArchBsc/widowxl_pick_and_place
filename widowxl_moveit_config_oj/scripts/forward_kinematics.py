import numpy as np

# --- Joint Angles (radians) ---
waist = np.radians(1)  # Z-axis (Yaw)
shoulder = np.radians(2)  # Y-axis (Pitch)
elbow = np.radians(23)  # Y-axis (Pitch)
wrist_angle = np.radians(68)  # Y-axis (Pitch)
wrist_rotate = np.radians(0)  # Z-axis (Roll)

# --- Compute Rotation Matrices ---
# Rotation about Z (waist)
R_waist = np.array([
    [np.cos(waist), -np.sin(waist), 0],
    [np.sin(waist), np.cos(waist), 0],
    [0, 0, 1]
])

# Rotation about Y (shoulder, elbow, wrist angle)
R_shoulder = np.array([
    [np.cos(shoulder), 0, np.sin(shoulder)],
    [0, 1, 0],
    [-np.sin(shoulder), 0, np.cos(shoulder)]
])

R_elbow = np.array([
    [np.cos(elbow), 0, np.sin(elbow)],
    [0, 1, 0],
    [-np.sin(elbow), 0, np.cos(elbow)]
])

R_wrist = np.array([
    [np.cos(wrist_angle), 0, np.sin(wrist_angle)],
    [0, 1, 0],
    [-np.sin(wrist_angle), 0, np.cos(wrist_angle)]
])

# Rotation about Z (wrist rotate)
R_wrist_rotate = np.array([
    [np.cos(wrist_rotate), -np.sin(wrist_rotate), 0],
    [np.sin(wrist_rotate), np.cos(wrist_rotate), 0],
    [0, 0, 1]
])

# --- Compute Final Orientation Matrix ---
R_final = R_waist @ R_shoulder @ R_elbow @ R_wrist @ R_wrist_rotate

# --- Extract Euler Angles (Yaw, Pitch, Roll) ---
yaw = np.arctan2(R_final[1, 0], R_final[0, 0])  # Z-axis rotation
pitch = np.arcsin(-R_final[2, 0])  # Y-axis rotation
roll = np.arctan2(R_final[2, 1], R_final[2, 2])  # X-axis rotation

# --- Convert to Degrees ---
yaw_deg = np.degrees(yaw)
pitch_deg = np.degrees(pitch)
roll_deg = np.degrees(roll)

# --- Display Results ---
print(f"✅ End-Effector Orientation (Euler Angles):")
print(f"   - Yaw (Z)   : {yaw_deg:.2f}°")
print(f"   - Pitch (Y) : {pitch_deg:.2f}°")
print(f"   - Roll (X)  : {roll_deg:.2f}°")


