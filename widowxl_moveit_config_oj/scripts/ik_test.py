#!/usr/bin/env python2
# -*- coding: utf-8 -*-


import time
import signal
from interbotix_xs_modules.arm import InterbotixManipulatorXS

def interpolate_positions(start, end, steps=200):
    """ Γεννά ομαλά ενδιάμεσα σημεία μεταξύ δύο καταστάσεων της τελικής θέσης """
    return [[start[i] + (end[i] - start[i]) * (t / steps) for i in range(len(start))] for t in range(steps + 1)]

def signal_handler(sig, frame):
    print("\n[INFO] Program interrupted! Exiting without moving arm...")
    exit(0)

def main():
    global bot
    bot = InterbotixManipulatorXS("widowxl", moving_time=5.0, accel_time=2.5)  # Μέτρια ταχύτητα
    
    # Καταγραφή του CTRL+C για ασφαλές κλείσιμο
    signal.signal(signal.SIGINT, signal_handler)
    
    pose_matrix = bot.arm.get_ee_pose()
    initial_pose = [pose_matrix[0][3], pose_matrix[1][3], pose_matrix[2][3]]
    if isinstance(pose, (list, tuple)) and len(pose) >= 3:
        initial_pose = [float(pose[0]), float(pose[1]), float(pose[2])]
    else:
        raise ValueError("Unexpected pose format from get_ee_pose(): {}".format(pose))
    if isinstance(pose, (list, tuple)):
        pose = [float(value) for value in pose]
    initial_pose = pose[:3]  # Παίρνει την τρέχουσα θέση ως αρχική
    bot.arm.set_ee_pose_components(x=initial_pose[0], y=initial_pose[1], z=initial_pose[2], blocking=True)
    bot.gripper.open()
    time.sleep(2)
    
    waypoints = [
        [0.25, -0.15, 0.25],
        [0.2, 0.05, 0.3],
        [0.35, 0.1, 0.35]
    ]
    
    current_pose = initial_pose
    for target_pose in waypoints:
        interpolated_positions = interpolate_positions(current_pose, target_pose, steps=200)  # Περισσότερα μικρά βήματα για αργή κίνηση
        for position in interpolated_positions:
            bot.arm.set_ee_pose_components(x=position[0], y=position[1], z=position[2], blocking=True)
            time.sleep(0.02)  # Επιβράδυνση κάθε μικρής κίνησης χωρίς κενά μεταξύ των κινήσεων
        current_pose = target_pose
    
    final_pose = initial_pose
    interpolated_positions = interpolate_positions(current_pose, final_pose, steps=200)
    for position in interpolated_positions:
        bot.arm.set_ee_pose_components(x=position[0], y=position[1], z=position[2], blocking=True)
        time.sleep(0.02)
    
if __name__=='__main__':
    main()

