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

    print("--- ΒΗΜΑ 3: Κίνηση 'Τετράγωνο' στον αέρα ---")
    # Θα κάνουμε ένα τετράγωνο κινώντας τον End-Effector (το χέρι).
    # Χρησιμοποιούμε το 'set_ee_pose_components' από το arm.py.
    # Ορίζουμε X, Y, Z σε μέτρα σε σχέση με τη βάση του ρομπότ.
    
    # Ύψος που θα γίνει η κίνηση (Z)
    height = 0.2 
    # Απόσταση μπροστά (X)
    x_pos = 0.3

    # Σημείο Α (Κέντρο)
    bot.arm.set_ee_pose_components(x=x_pos, y=0.0, z=height, pitch=1.5)
    
    # Σημείο Β (Αριστερά - Y θετικό)
    bot.arm.set_ee_pose_components(x=x_pos, y=0.1, z=height, pitch=1.5)
    
    # Σημείο Γ (Μπροστά και Αριστερά - X πιο μακριά)
    bot.arm.set_ee_pose_components(x=x_pos + 0.05, y=0.1, z=height, pitch=1.5)
    
    # Σημείο Δ (Μπροστά και Δεξιά - Y αρνητικό)
    bot.arm.set_ee_pose_components(x=x_pos + 0.05, y=-0.1, z=height, pitch=1.5)

    # Σημείο Ε (Δεξιά - X επιστροφή)
    bot.arm.set_ee_pose_components(x=x_pos, y=-0.1, z=height, pitch=1.5)
    
    # Επιστροφή στο Κέντρο
    bot.arm.set_ee_pose_components(x=x_pos, y=0.0, z=height, pitch=1.5)

    print("--- ΒΗΜΑ 4: Παρκάρισμα (Sleep) ---")
    # Το ρομπότ διπλώνει σε θέση ασφαλείας και χαλαρώνει τα μοτέρ
    # Συνάρτηση από: arm.py
    bot.arm.go_to_sleep_pose()

if __name__ == '__main__':
    main()
