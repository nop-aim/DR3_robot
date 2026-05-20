import kinematic.inv_kinematic as IK
import kinematic.forward_kinematics as FK
import com.connect as cn
import numpy as np
import time
import queue

if __name__ == "__main__":

    log_file = open("arduino_log.txt", "w", encoding="utf-8")
    cn.open_serial()
    theta_cur = [0, 0, 0]

    while True:
        user_input = input("\nNhập lệnh: ").strip()

        if user_input.lower() == "q":
            print("Thoát chương trình.")
            log_file.close()
            break

        # ===============================
        # XỬ LÝ LỆNH KHÔNG PHẢI TỌA ĐỘ
        # ===============================
        if user_input.upper() == "HOME":
            cn.send_home_cmd()
            target_cmd = "HOME"

        elif user_input.upper() == "HUT":
            cn.send_hut_cmd()
            target_cmd = "HUT"

        elif user_input.upper() == "THA":
            cn.send_tha_cmd()
            target_cmd = "THA"

        # ===============================
        # XỬ LÝ LỆNH TỌA ĐỘ
        # ===============================
        else:
            try:
                Px, Py, Pz = map(float, user_input.split())
            except:
                print("Sai định dạng. Nhập: Px Py Pz")
                continue

            target_xyz = (Px, Py, Pz)

            # IK
            best, ok, length, diff = IK.inv_kinematic(target_xyz, theta_cur)
            if not ok:
                print("Không tìm được nghiệm IK.")
                continue

            w1, w2, w3 = best
            print(f"Gửi góc: {w1:.2f}, {w2:.2f}, {w3:.2f}")

            cn.send_set_angles_cmd(w1, w2, w3)
            target_cmd = "SET"

        # ============================================================
        # CHỜ ARDUINO XỬ LÝ LỆNH
        # ============================================================
        print("Đang chờ Arduino...")

        start_time = time.time()
        timeout =  None     
        done = False
        while not done:

            # ------------------- WAIT FOR HOME -------------------
            if target_cmd == "HOME":
                if cn.wait_for_home_done():
                    print("Robot đã về HOME.")
                    done = True
                else:
                    time.sleep(0.01)

            # ------------------- WAIT FOR HUT -------------------
            elif target_cmd == "HUT":
                if cn.wait_for_hut_done():
                    print("Robot đã HUT.")
                    done = True
                else:
                    time.sleep(0.01)

            # ------------------- WAIT FOR THA -------------------
            elif target_cmd == "THA":
                if cn.wait_for_tha_done():
                    print("Robot đã THA.")
                    done = True
                else:
                    time.sleep(0.01)

            # ------------------- WAIT FOR SET -------------------
            elif target_cmd == "SET":
                try:
                    line = cn.serial_queue.get_nowait()
                    if line == "DONE":
                        print("Arduino đã hoàn thành lệnh SET.")
                        done = True
                    else:
                        # Print log nếu cần
                        print("[ARD LOG]", line)
                        log_file.write(line + "\n")
                        log_file.flush()

                except queue.Empty:
                    pass

                time.sleep(0.01)

            # ------------------- TIMEOUT -------------------
            #if time.time() - start_time > timeout:
            #   print("TIMEOUT! Arduino không phản hồi.")
            #   done = True

        # ======================================================H======
        # XỬ LÝ SAU KHI HOÀN THÀNH LỆNH SET
        # ============================================================
        if target_cmd == "SET":
            T0EE = FK.forward_kinematic(w1, w2, w3, degree=True)
            Px_calc, Py_calc, Pz_calc = T0EE[0,3], T0EE[1,3], T0EE[2,3]

            print(f" → DK tiến: Px={Px_calc:.2f}, Py={Py_calc:.2f}, Pz={Pz_calc:.2f}\n")

            # Update state
            theta_cur = [w1, w2, w3]

