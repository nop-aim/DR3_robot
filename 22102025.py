import tkinter as tk
from PIL import Image, ImageTk
import numpy as np
import serial
import time
import threading
import math

# ---------------- Serial ----------------
SERIAL_PORT = 'COM7'
BAUDRATE = 9600
ser = None
serial_lock = threading.Lock()

def open_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        time.sleep(2)
        print("Serial da mo:", SERIAL_PORT)
    except Exception as e:
        ser = None
        print("Khong mo duoc serial:", e)

open_serial()

def send_serial_line(line):
    with serial_lock:
        if ser and ser.is_open:
            try:
                ser.write((line + "\n").encode('utf-8'))
                print("Gui:", line)
                start = time.time()
                while time.time() - start < 0.5:
                    if ser.in_waiting:
                        resp = ser.readline().decode(errors='ignore').strip()
                        if resp:
                            print("Arduino:", resp)
                    else:
                        time.sleep(0.02)
            except Exception as e:
                print("Loi gui serial:", e)
        else:
            print("Serial chua mo:", line)

def send_set_angles_cmd(t1, t2, t3):
    cmd = f"SET {t1:.2f} {t2:.2f} {t3:.2f}"
    send_serial_line(cmd)

def send_home_cmd():
    send_serial_line("HOME")

# ---------------- Forward Kinematics ----------------
def fk(theta_deg):
    t1, t2, t3 = np.deg2rad(theta_deg)
    px = (2291*np.cos(t1))/20 - (111*np.sin(t1))/2 + 162*np.cos(t1)*np.cos(t2) - 149*np.cos(t1)*np.sin(t2)*np.sin(t3) + 149*np.cos(t1)*np.cos(t2)*np.cos(t3)
    py = (111*np.cos(t1))/2 + (2291*np.sin(t1))/20 + 162*np.cos(t2)*np.sin(t1) - 149*np.sin(t1)*np.sin(t2)*np.sin(t3) + 149*np.cos(t2)*np.cos(t3)*np.sin(t1)
    pz = 162*np.sin(t2) + 149*np.cos(t2)*np.sin(t3) + 149*np.cos(t3)*np.sin(t2) + 5231/20
    return np.array([px, py, pz], dtype=float)

# ---------------- Inverse Kinematics ----------------
def ik_solve(target_xyz, theta_init_deg):
    Px, Py, Pz = target_xyz
    l1, l2, l3 = 114.55, 162.0, 145.05
    d1, d2, d3 = 261.55, 104.0, 48.5

    def ang_norm_deg(a):
        return ((a + 180.0) % 360.0) - 180.0

    def angle_diff_sum(sol, cur):
        diff = ((sol - cur + 180.0) % 360.0) - 180.0
        return np.sum(np.abs(diff))

    a, b = -Py, Px
    d = d3 - d2
    r = np.hypot(a, b)
    if r == 0 or abs(d / r) > 1.0:
        return np.array(theta_init_deg), False, 0, float(np.linalg.norm(target_xyz - fk(theta_init_deg)))

    alpha = math.atan2(a, b)
    root_term = np.sqrt(max(0.0, 1.0 - (d**2) / (r**2)))
    theta1_a = math.atan2(d / r, root_term) - alpha
    theta1_b = math.atan2(d / r, -root_term) - alpha

    def solve_branch(theta1_rad):
        E = Px * math.cos(theta1_rad) + Py * math.sin(theta1_rad) - l1
        F = -(Pz - d1)
        C = E**2 + F**2 - l2**2 - l3**2
        c3 = C / (2*l2*l3)
        if abs(c3) > 1.0:
            return []
        s3_vals = [np.sqrt(1 - c3**2), -np.sqrt(1 - c3**2)]
        sols = []
        for s3 in s3_vals:
            theta3 = math.atan2(s3, c3)
            denom = (l3*c3 + l2)**2 + (l3*s3)**2
            c2 = (E*(l3*c3 + l2) + l3*s3*F) / denom
            s2 = (F*(l3*c3 + l2) - l3*s3*E) / denom
            theta2 = math.atan2(s2, c2)
            sols.append([theta1_rad, theta2, theta3])
        return sols

    sols = solve_branch(theta1_a) + solve_branch(theta1_b)
    if not sols:
        return np.array(theta_init_deg), False, 0, 999

    sols_deg = [np.degrees(sol) for sol in sols if not np.any(np.isnan(sol))]
    valid = [np.array([ang_norm_deg(x) for x in sol]) for sol in sols_deg]
    if not valid:
        return np.array(theta_init_deg), False, 0, 999

    diffs = [angle_diff_sum(v, theta_init_deg) for v in valid]
    best = valid[np.argmin(diffs)]
    err = float(np.linalg.norm(target_xyz - fk(best)))
    return best, True, len(valid), err

# ---------------- GUI ----------------
class RobotApp:
    def __init__(self, root):
        self.root = root
        root.title("ROBOT IK CONTROL")
        root.geometry("1150x650")
        root.configure(bg="#0f172a")  # dark tech background

        # Header
        header = tk.Label(
            root, text="ROBOT CONTROL PANEL",
            font=("Consolas", 18, "bold"), fg="#00eaff", bg="#0f172a"
        )
        header.pack(pady=10)

        # Main container with subtle border
        self.theta = np.array([0.0, 0.0, 0.0])
        self.cell_buttons = {}
        frm = tk.Frame(root, bg="#0b1220", bd=2, relief="groove")
        frm.pack(padx=15, pady=10, fill='both', expand=True)

        # Angle entries
        tk.Label(frm, text="Theta1 (deg):", bg="#0b1220", fg="white").grid(row=0, column=0, sticky='w', padx=6, pady=2)
        self.e_t1 = tk.Entry(frm, width=8)
        self.e_t1.grid(row=0, column=1)
        tk.Label(frm, text="Theta2 (deg):", bg="#0b1220", fg="white").grid(row=1, column=0, sticky='w', padx=6, pady=2)
        self.e_t2 = tk.Entry(frm, width=8)
        self.e_t2.grid(row=1, column=1)
        tk.Label(frm, text="Theta3 (deg):", bg="#0b1220", fg="white").grid(row=2, column=0, sticky='w', padx=6, pady=2)
        self.e_t3 = tk.Entry(frm, width=8)
        self.e_t3.grid(row=2, column=1)

        tk.Button(frm, text="Set", command=self.set_angles, bg="#24303a", fg="white").grid(row=3, column=0, columnspan=2, pady=6)
        tk.Button(frm, text="HOME", bg="#ff6666", fg="white", command=self.home_pressed).grid(row=4, column=0, columnspan=2)

        # Target coordinates
        tk.Label(frm, text="Target X:", bg="#0b1220", fg="white").grid(row=5, column=0, sticky='w', padx=6, pady=2)
        self.e_x = tk.Entry(frm, width=10)
        self.e_x.grid(row=5, column=1)
        tk.Label(frm, text="Target Y:", bg="#0b1220", fg="white").grid(row=6, column=0, sticky='w', padx=6, pady=2)
        self.e_y = tk.Entry(frm, width=10)
        self.e_y.grid(row=6, column=1)
        tk.Label(frm, text="Target Z:", bg="#0b1220", fg="white").grid(row=7, column=0, sticky='w', padx=6, pady=2)
        self.e_z = tk.Entry(frm, width=10)
        self.e_z.grid(row=7, column=1)
        tk.Button(frm, text="Run IK", bg="#16a34a", fg="black", command=self.run_ik_thread).grid(row=8, column=0, columnspan=2, pady=10)

        self.status = tk.Label(frm, text="San sang", anchor='w', bg="#0b1220", fg="#00eaff")
        self.status.grid(row=9, column=0, columnspan=2)
        self.pos_label = tk.Label(frm, text="Pos: ---", bg="#0b1220", fg="#bbf7d0")
        self.pos_label.grid(row=10, column=0, columnspan=2)

        # Coordinate board
        grid_frame = tk.LabelFrame(frm, text="BAN TOA DO", padx=12, pady=8, bg="#0b1220", fg="#00eaff", bd=1)
        grid_frame.grid(row=0, column=3, rowspan=11, padx=60)

        self.cell_coords = {
            "E3": [197.5+5, 131.5-3, 21], "E2": [232.5+5, 131.5-4, 22], "E1": [267.5+5, 131.5-5, 23],
            "D3": [197.5+5, 94.5-3, 21], "D2": [232.5+5, 94.5-3, 22], "D1": [267.5+5, 94.5-3, 20],
            "C3": [197.5+5, 59.5-3, 21], "C2": [232.5+5, 59.5-3, 20], "C1": [267.5+5, 59.5-3, 20],
            "B3": [197.5+5, 24.5-3, 20], "B2": [232.5+5, 24.5-3, 20], "B1": [267.5+5, 24.5-3, 19],
            "A3": [197.5+5, -10.5-3, 20], "A2": [232.5+5, -10.5-3, 20], "A1": [267.5+5, -10.5-3, 18],
        }

        for r, row_label in enumerate(['1', '2', '3']):
            for c, col_label in enumerate(['E', 'D', 'C', 'B', 'A']):
                name = f"{col_label}{row_label}"
                b = tk.Button(
                    grid_frame, text=name, width=6, height=2,
                    command=lambda n=name: self.cell_clicked(n),
                    bg="#243342", fg="white", activebackground="#22d3ee"
                )
                b.grid(row=r, column=c, padx=4, pady=3)
                self.cell_buttons[name] = b

        # Electromagnet & Box controls
        ctrl_frame = tk.LabelFrame(frm, text="NAM CHAM & BOX", padx=10, pady=10, bg="#0b1220", fg="#00eaff")
        ctrl_frame.grid(row=0, column=6, rowspan=11, padx=10)
        tk.Button(ctrl_frame, text="HUT", width=8, bg="#38bdf8", fg="black", command=self.hut_pressed).grid(row=0, column=1, pady=5)
        tk.Button(ctrl_frame, text="THA", width=8, bg="#fb7185", fg="black", command=self.tha_pressed).grid(row=1, column=1, pady=5)
        tk.Button(ctrl_frame, text="BOX", width=8, bg="#94a3b8", fg="black", command=self.box_pressed).grid(row=2, column=1, pady=5)

        # startup to home
        self.go_home_startup()

    # ================= CORE FUNCTION =================
    def _home_worker(self):
        send_home_cmd()
        self.theta = np.array([0.0, 0.0, 0.0])
        self.root.after(100, self.update_angle_entries)

    def go_home_startup(self):
        self.status.config(text="Dang ve HOME...")
        threading.Thread(target=self._home_worker, daemon=True).start()

    def set_angles(self):
        try:
            t1, t2, t3 = float(self.e_t1.get()), float(self.e_t2.get()), float(self.e_t3.get())
            threading.Thread(target=send_set_angles_cmd, args=(t1, t2, t3), daemon=True).start()
            self.theta = np.array([t1, t2, t3])
            self.status.config(text="Da gui goc")
            self.update_angle_entries()
        except Exception as e:
            self.status.config(text=f"Loi set goc: {e}")

    def update_angle_entries(self):
        for e, v in zip([self.e_t1, self.e_t2, self.e_t3], self.theta):
            e.delete(0, tk.END)
            e.insert(0, f"{v:.2f}")
        pos = fk(self.theta)
        self.pos_label.config(text=f"Pos: {pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}")
        self.status.config(text="San sang")

    def home_pressed(self):
        self.status.config(text="Dang ve HOME...")
        threading.Thread(target=self._home_worker, daemon=True).start()

    def run_ik_thread(self):
        threading.Thread(target=self.run_ik, daemon=True).start()

    def run_ik(self):
        try:
            x, y, z = float(self.e_x.get()), float(self.e_y.get()), float(self.e_z.get())
        except:
            self.status.config(text="Loi nhap toa do")
            return

        self.status.config(text="Dang chay IK...")
        theta_sol, ok, nsol, err = ik_solve(np.array([x, y, z]), self.theta)
        if ok:
            self.status.config(text=f"IK ok (err={err:.3f})")
            self.theta = theta_sol
            send_set_angles_cmd(*self.theta)
            self.root.after(100, self.update_angle_entries)
        else:
            self.status.config(text=f"IK loi (err={err:.3f})")

    # ================= MOVE WITH Z SAFE =================
    def move_to_cell(self, name, xyz):
        try:
            self.status.config(text=f"Dang di chuyen den {name}...")
            x, y, z_final = xyz
            z_safe = 35

            mid_xyz = np.array([x, y, z_safe])
            theta_mid, ok_mid, _, _ = ik_solve(mid_xyz, self.theta)
            if ok_mid:
                self.theta = theta_mid
                send_set_angles_cmd(*self.theta)
                self.root.after(100, self.update_angle_entries)
                time.sleep(0.1)

            final_xyz = np.array([x, y, z_final])
            theta_final, ok_final, _, _ = ik_solve(final_xyz, self.theta)
            if ok_final:
                self.theta = theta_final
                send_set_angles_cmd(*self.theta)
                self.root.after(100, self.update_angle_entries)
                time.sleep(0.1)
                self.status.config(text=f"Da den {name}")
            else:
                self.status.config(text=f"Khong toi duoc {name}")
        except Exception as e:
            self.status.config(text=f"Loi khi di chuyen: {e}")

    def cell_clicked(self, name):
        for btn in self.cell_buttons.values():
            btn.config(bg='#243342')
        self.cell_buttons[name].config(bg='#facc15')
        threading.Thread(target=self.move_to_cell, args=(name, self.cell_coords[name]), daemon=True).start()

    # ================= NAM CHAM =================
    def hut_pressed(self):
        send_serial_line("HUT")
        self.status.config(text="Nam cham ON")

    def tha_pressed(self):
        send_serial_line("THA")
        self.status.config(text="Nam cham OFF")

    def box_pressed(self):
        threading.Thread(target=self.goto_box, daemon=True).start()

    def goto_box(self):
        try:
            self.status.config(text="Dang di toi BOX...")
            x, y, z_final = 207.5, 200.0, 30
            z_safe = 40

            mid_xyz = np.array([x, y, z_safe])
            theta_mid, ok_mid, _, _ = ik_solve(mid_xyz, self.theta)
            if ok_mid:
                self.theta = theta_mid
                send_set_angles_cmd(*self.theta)
                self.root.after(100, self.update_angle_entries)
                time.sleep(0.1)

            final_xyz = np.array([x, y, z_final])
            theta_final, ok_final, _, _ = ik_solve(final_xyz, self.theta)
            if ok_final:
                self.theta = theta_final
                send_set_angles_cmd(*self.theta)
                self.root.after(100, self.update_angle_entries)
                time.sleep(0.1)
                self.status.config(text="Da toi BOX")
            else:
                self.status.config(text="Khong toi duoc BOX")
        except Exception as e:
            self.status.config(text=f"Loi khi di toi BOX: {e}")

# ================= MAIN APP =================
def open_control_board(welcome_window):
    welcome_window.destroy()
    control_root = tk.Tk()
    app = RobotApp(control_root)

    def on_close():
        if ser and ser.is_open:
            try:
                ser.close()
                print("Serial dong.")
            except Exception as e:
                print("Loi dong serial:", e)
        control_root.destroy()

    control_root.protocol("WM_DELETE_WINDOW", on_close)
    control_root.mainloop()

if __name__ == "__main__":
    giaodien = tk.Tk()
    giaodien.title("NHOM 1 - THUC TAP ROBOT")
    giaodien.geometry("1400x800")

    try:
        image_path = r"E:\TT_ROBOT\CODE\PYTHON\giaodien.jpg"
        original_image = Image.open(image_path)
        resized_image = original_image.resize((1350, 750), Image.LANCZOS)
        photo = ImageTk.PhotoImage(resized_image)
        image_label = tk.Label(giaodien, image=photo)
        image_label.pack(pady=10)
    except Exception as e:
        print(f"Loi anh: {e}")
        tk.Label(giaodien, text="Khong tim thay anh!", font=("Arial", 24), fg="red").pack(pady=200)

    tk.Button(
        giaodien, text="TIEP THEO", font=("Arial", 21),
        bg="green", fg="white",
        command=lambda: open_control_board(giaodien)
    ).place(x=1200, y=650)

    if 'photo' in locals():
        image_label.image = photo

    giaodien.mainloop()
