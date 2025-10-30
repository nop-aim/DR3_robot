import numpy as np
import kinematic.forward_kinematics as FK
def inv_kinematic(target_xyz,theta_cur):
    """
    Tính nghịch đảo động học cho robot 3 bậc tự do.

    Tham số:
    Px, Py, Pz: Tọa độ điểm cần đạt tới (mm)

    Trả về:
    Danh sách các bộ nghiệm (w1, w2, w3) tính bằng radian.
    """
    # === Thông số robot ===
    Px,Py,Pz = target_xyz
    L1 = 180
    L2 = 244
    L3 = 269
    d1 = 120
    d2 = 93
    # === Hàm chuẩn hóa góc ===
    def ang_norm_deg(a):
        return ((a + 180.0) % 360.0) - 180.0

    # === Hàm tính tổng độ lệch góc ===
    def angle_diff_sum(sol, cur):
        diff = ((sol - cur + 180.0) % 360.0) - 180.0
        return np.sum(np.abs(diff))
    # ==== Tính w3 ====
    A = Px**2 + Py**2 - ((d2 - d1)**2 + L3**2 + L2**2 - (Pz - L1)**2)
    B = 2 * L2 * L3
    c3 = A / B
    if abs(c3) > 1:
        raise ValueError("Không có nghiệm thực cho w3")

    s3 = np.sqrt(1 - c3**2)
    s3_two = -s3
    w3 = np.arctan2(s3, c3)
    w3_two = np.arctan2(s3_two, c3)

    # ==== Tính w2 cho từng w3 ====
    a1 = L3 * np.sin(w3)
    b1 = L3 * np.cos(w3) + L2
    c = Pz - L1
    R1 = np.sqrt(a1**2 + b1**2)
    phi1 = np.arctan2(b1, a1)

    a2 = L3 * np.sin(w3_two)
    b2 = L3 * np.cos(w3_two) + L2
    R2 = np.sqrt(a2**2 + b2**2)
    phi2 = np.arctan2(b2, a2)

    if abs(c) < R1:
        w2 = phi1 + np.arccos(c / R1)
        w2_two = phi1 - np.arccos(c / R1)
    else:
        print("Không có nghiệm w2")
        w2 = None
        w2_two = None

    if abs(c) < R2:
        w22 = phi2 + np.arccos(c / R2)
        w22_two = phi2 - np.arccos(c / R2)
    else:
        print("Không có nghiệm w22")
        w22 = None
        w22_two = None

    # ==== Tính w1 cho từng cặp (w2, w3) ====
    B = d2 - d1
    solutions = []

    # Duyệt 4 cặp (w2, w3)
    for w3_now in [w3, w3_two]:
        if np.isclose(w3_now, w3):
            w2_list = [w2, w2_two]
        else:
            w2_list = [w22, w22_two]

        for w2_now in w2_list:
            A = L2 * np.cos(w2_now) + L3 * np.cos(w2_now + w3_now)
            R = np.sqrt(A**2 + B**2)
            phi = np.arctan2(B, A)
        if abs(Px) <= R:
                try:
                    w1_1 = phi + np.arccos(Px / R)
                    w1_2 = phi - np.arccos(Px / R)
                    # === Kiểm chứng Py để chọn dấu đúng ===
                    # Tính Py dự đoán lại từ từng w1 ứng viên
                    Py_calc_1 = A * np.sin(w1_1) - B * np.cos(w1_1)
                    Py_calc_2 = A * np.sin(w1_2) - B * np.cos(w1_2)

                    # Chọn nghiệm nào cho Py_calc gần với Py thật
                    if abs(Py - Py_calc_1) < abs(Py - Py_calc_2):
                        solutions.append([w1_1, w2_now, w3_now])
                    else:
                        solutions.append([w1_2, w2_now, w3_now])
                except ValueError:
                    # Khi Px/R vượt ngoài [-1,1]
                    continue

    # ==== Loại nghiệm trùng (theo độ) ====
    solutions = np.array(solutions)
    sol_deg = np.round(np.degrees(solutions), 4) #round làm tròn đến số thập phân thứ 4
    sols_deg = np.unique(sol_deg, axis=0) #loại bỏ các phần tử trung nhau axic = 0 là so sanh theo hàng 
    for i, sol in enumerate(sols_deg):
        print(f" Nghiệm {i+1}: (w1, w2, w3) = ({sol[0]:.2f}°, {sol[1]:.2f}°, {sol[2]:.2f}°)")
        # Kiểm tra bằng cách tính động học tiến
        T0EE = FK.forward_kinematic(sol[0], sol[1], sol[2], True)
        p_calc = T0EE[0:3, 3]
        print(f"  Tọa độ tính được từ động học tiến: Px={p_calc[0]:.2f}, Py={p_calc[1]:.2f}, Pz={p_calc[2]:.2f}\n")
    # ==== Chuẩn hóa góc về [-180, 180] độ ====
    valid = [np.array([ang_norm_deg(x) for x in sol]) for sol in sols_deg]
    diffs = [angle_diff_sum(v, theta_cur) for v in valid]
    best = valid[np.argmin(diffs)]
    print(f" Nghiệm tốt nhất: (w1, w2, w3) = ({best[0]:.2f}°, {best[1]:.2f}°, {best[2]:.2f}°)")
    print(f"Δ Góc tổng = {diffs[np.argmin(diffs)]:.3f}°")
    return best, True, len(valid), diffs[np.argmin(diffs)] 
    