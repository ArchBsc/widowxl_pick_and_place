import sys
# Εισάγουμε την κλάση που είδαμε στο arm.py
from interbotix_xs_modules.arm import InterbotixManipulatorXS

# --- ΡΥΘΜΙΣΕΙΣ ---
# ΑΛΛΑΞΕ ΤΟ 'wx250s' ΜΕ ΤΟ ΔΙΚΟ ΣΟΥ ΜΟΝΤΕΛΟ (π.χ. 'vx300', 'px100')
ROBOT_MODEL = "widowxl" 

def main():
    # 1. Αρχικοποίηση (Δημιουργία του αντικειμένου bot)
    # Αυτό σηκώνει το 'core.py' και συνδέεται με το ROS.
    bot = InterbotixManipulatorXS(
        robot_model=ROBOT_MODEL,
        group_name="arm",
        gripper_name="gripper"
    )
    
    print("--- ΒΗΜΑ 1: Homing ---")
    # Πάει το ρομπότ στην αρχική θέση (όλες οι αρθρώσεις στο 0 εκτός από κάποιες default)
    # Συνάρτηση από: arm.py
    bot.arm.go_to_home_pose()

    print("--- ΒΗΜΑ 2: Έλεγχος Δαγκάνας ---")
    # Ανοίγει και κλείνει τη δαγκάνα για να δούμε ότι δουλεύει
    # Συναρτήσεις από: gripper.py
    bot.gripper.open()
    bot.gripper.close()
    bot.gripper.open() # Την αφήνουμε ανοιχτή

    bot.arm.go_to_sleep_pose()

if __name__ == '__main__':
    main()
