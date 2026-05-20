import numpy as np
import kinematic.forward_kinematics as FK

def inv_kinematic(target_xyz, theta_cur):
    """
    Nghịch đảo động học cho robot 3 DOF
    Trả về 4 nghiệm (hoặc ít hơn nếu không tồn tại).
    """

    # --- Robot parameters ---
    Px, Py, Pz = target_xyz
    L1 = 142+180
    L2 = 275
    L3 = 280
    d1 = 135
    d2 = 115
    # --- Normalize angle [-180,180] ---
    def ang_norm_deg(a):
        return (a % 360)

    # --- Angle distance ---
    def angle_diff_sum(sol, cur):
        diff = ((sol - cur + 180) % 360) - 180
        return np.sum(np.abs(diff))

    # -----------------------------------------
    # 1) TÍNH W1
    # -----------------------------------------
    r = np.hypot(Px, -Py)
    if r == 0:
        print("Không tính được w1 (r=0)")
        return [], False, 0, None

    alpha = np.arctan2(Px, -Py)
    t = np.clip((d2 - d1) / r, -1, 1)
    sq = np.sqrt(max(0, 1 - t*t))

    w1_candidates = [
        np.arctan2( sq, t) + alpha,
        np.arctan2(-sq, t) + alpha
    ]

    # -----------------------------------------
    # 2) Duyệt từng w1 → w2, w3
    # -----------------------------------------
    solutions = []

    for w1 in w1_candidates:

        # Tính A, B cho w3/w2
        A = Px*np.cos(w1) + Py*np.sin(w1)
        B = Pz - L1

        # c3
        c3 = np.clip((A*A + B*B - L2*L2 - L3*L3) / (2*L2*L3), -1, 1)
        s3 = np.sqrt(max(0, 1 - c3*c3))

        # Hai nhánh w3
        for s3_sign in [s3, -s3]:
            w3 = np.arctan2(s3_sign, c3)

            # Tính w2
            num = B
            den = A

            w2_temp = np.arctan2(num, den) - np.arctan2(L2 + L3*np.cos(w3), -L3*np.sin(w3))
            w2 = np.arctan2(np.sin(w2_temp), np.cos(w2_temp))   # normalize (-pi,pi]

            solutions.append([w1, w2, w3])

    # =========================================
    # 3) LOẠI NGHIỆM TRÙNG
    # =========================================
    solutions = np.array(solutions)
    sol_deg = np.round(np.degrees(solutions), 4)
    sols_deg = np.unique(sol_deg, axis=0)

    # In nghiệm và kiểm tra FK
    for i, sol in enumerate(sols_deg):
        print(f" Nghiệm {i+1}: w1={sol[0]:.2f}°, w2={sol[1]:.2f}°, w3={sol[2]:.2f}°")
        T = FK.forward_kinematic(sol[0], sol[1], sol[2], True)
        p = T[:3, 3]
        print(f"   FK → Px={p[0]:.2f}, Py={p[1]:.2f}, Pz={p[2]:.2f}")

    # =========================================
    # 4) CHỌN NGHIỆM CẢ 3 ĐỀU ÂM
    # =========================================
    for sol in sols_deg:
        if all(sol <= 0):
            print(f" Chọn nghiệm cả 3 góc đều âm: w1={sol[0]:.2f}°, w2={sol[1]:.2f}°, w3={sol[2]:.2f}°")
            return sol, True, len(sols_deg), None
    