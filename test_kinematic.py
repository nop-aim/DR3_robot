import kinematic.inv_kinematic as IK
import numpy as np
import kinematic.forward_kinematics as FK

if __name__ == "__main__":
    # Tọa độ điểm cần đạt tới
    Px = 150
    Py = -70
    Pz = 180
    target_xyz = (Px, Py, Pz)
    theta_cur = [0, 0, 0]  # Giả sử robot đang ở vị trí ban đầu
    # Tính nghịch đảo động học
    best,ok,len,diff = IK.inv_kinematic(target_xyz, theta_cur)

    # In ra các bộ nghiệm tìm được
    if ok:
            w1, w2, w3 = best
            print(f"Bộ nghiệm: w1={w1:.2f}°, w2={w2:.2f}°, w3={w3:.2f}°")
            # Kiểm tra bằng cách tính động học tiến
            T0EE = FK.forward_kinematic(w1, w2, w3, degree=True)
            Px_calc = T0EE[0, 3]
            Py_calc = T0EE[1, 3]
            Pz_calc = T0EE[2, 3]
            print(f"  Tọa độ tính được từ động học tiến: Px={Px_calc:.2f}, Py={Py_calc:.2f}, Pz={Pz_calc:.2f}\n")