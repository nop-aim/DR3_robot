import queue
import time
# Sửa import để chạy từ main.py
from kinematic import inv_kinematic as IK
from com import connect as cn

class RobotController:
    def __init__(self, command_queue, response_queue):
        self.cmd_queue = command_queue
        self.res_queue = response_queue
        self.theta_cur = [0, 0, 0]
        self.running = True

    def run(self):
        try:
            cn.open_serial()
            self.res_queue.put(("LOG", "Kết nối Serial thành công."))
        except Exception as e:
            self.res_queue.put(("ERROR", "Kết nối Serial thất bại."))

        while self.running:
            try:
                cmd_tuple = self.cmd_queue.get(timeout=0.1)
                cmd_type, data = cmd_tuple
                if cmd_type == "QUIT":
                    self.running = False
                    break
                self.process_command(cmd_type, data)
            except queue.Empty:
                continue

    def process_command(self, cmd_type, data):
        if cmd_type in ["SET", "HOLE"]:
            px, py, pz, idx = data
            # Gọi hàm IK từ module kinematic
            best, ok, _, _ = IK.inv_kinematic((px, py, pz), self.theta_cur)
            
            if ok:
                angles = best
                ratios = [144, 48.0, 11.0]
                pulses = [a * r for a, r in zip(angles, ratios)]
                self.res_queue.put(("IK_SUCCESS", (angles, pulses, idx)))
                cn.send_set_angles_cmd(*angles)
                self.wait_for_done(cmd_type, idx, angles)
            else:
                self.res_queue.put(("IK_FAILED", None))

        elif cmd_type == "HOME":
            cn.send_home_cmd()
            if cn.wait_for_home_done():
                self.theta_cur = [0, 0, 0]
                self.res_queue.put(("DONE", ("HOME", -1, self.theta_cur)))

        elif cmd_type == "HUT":
            cn.send_hut_cmd()
            if cn.wait_for_hut_done():
                self.res_queue.put(("DONE", ("HUT", -1, None)))

        elif cmd_type == "THA":
            cn.send_tha_cmd()
            if cn.wait_for_tha_done():
                self.res_queue.put(("DONE", ("THA", -1, None)))

    def wait_for_done(self, cmd_type, idx, angles):
        start_time = time.time()
        while time.time() - start_time < 300: # Tăng timeout lên 15s
            try:
                line = cn.serial_queue.get(timeout=0.1).strip()
                if line == "ALL DONE":
                    self.theta_cur = angles
                    self.res_queue.put(("DONE", (cmd_type, idx, angles)))
                    return
            except queue.Empty:
                continue
        self.res_queue.put(("ERROR", "Timeout: Robot không phản hồi DONE"))