from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
import time

def main():
    bot = InterbotixManipulatorXS("widowxl", "arm", "gripper")

    bot.gripper.set_pressure(1)
    bot.arm.go_to_home_pose()
    bot.gripper.open()


    target_x = 0.20 
    target_y = -0.05
    target_z = -0.0605

    bot.arm.set_ee_pose_components(x=target_x, y=target_y, z=target_z + 0.05, pitch=1.5)


    bot.arm.set_ee_pose_components(x=target_x, y=target_y, z=target_z, pitch=1.5)
    bot.gripper.close()
    
    time.sleep(2)
    
    bot.arm.set_ee_pose_components(x=target_x, y=target_y, z=target_z + 0.1, pitch=1.5)
    
    
    bot.arm.go_to_home_pose()

if __name__ == '__main__':
    main()
