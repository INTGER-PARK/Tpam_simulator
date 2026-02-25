float64[3] pos
float64[3] vel
float64[3] acc
float64[3] rpy        # [roll, pitch, yaw]
float64[3] w_rpy      # [w_roll, w_pitch, w_yaw]
float64[3] a_rpy      # [a_roll, a_pitch, a_yaw]
float64[4] servo      # [Motor1_tilted_angle, Motor2_tilted_angle, Motor3_tilted_angle, Motor4_tilted_angle]

# --- add for 9D q = [base(6), arm(3)] ---
float64[3] arm_q     # [delta1, delta2, delta3]
float64[3] arm_dq    # [ddelta1, ddelta2, ddelta3]