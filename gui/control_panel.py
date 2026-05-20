import tkinter as tk
from tkinter import ttk
import queue

# --- CẤU HÌNH GIAO DIỆN ---
WINDOW_WIDTH = 960
WINDOW_HEIGHT = 600
HOLE_X, HOLE_Y, HOLE_Z = 235.0, -218.0, 240.0
Z1=217
Z2=198
Z3=193
class ControlPanel(tk.Toplevel):
    def __init__(self, master, cmd_queue, res_queue):
        super().__init__(master)
        self.cmd_queue = cmd_queue
        self.res_queue = res_queue
        
        self.title("BẢNG ĐIỀU KHIỂN ROBOT DR3")
        self.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        self.configure(bg="white")
        # Biến quản lý trạng thái
        self.input_index = 0
        self.execution_index = 0
        self.current_execution_index = -1
        
        # Danh sách lưu trữ Widget để cập nhật dữ liệu
        self.all_object_entries = []
        self.entry_deg = []
        self.entry_target = []
        self.entry_ec = []
        # --- AUTO HUT 5 VẬT 1 LẦN ---
        self.auto_running = False
        self.auto_job = []
        self.auto_step = 0
        # Delay theo từng loại lệnh (ms)
        self.delay_map = {
            "SET": 3000,
            "HUT": 100,
            "THA": 200,
            "HOME":3000
        }

        self.tray_coordinates = {
            # --- Cột A ---
            'A1': (HOLE_X - 100, HOLE_Y, Z1), 
            
            # --- Cột B ---
            'B1': (HOLE_X - 83, HOLE_Y + 83, 194), 
            'B2': (HOLE_X - 70, HOLE_Y + 27, Z3-1),
            'B3': (HOLE_X - 75, HOLE_Y, Z1),
            'B4': (HOLE_X - 81, HOLE_Y - 22, Z3),  
            'B5': (HOLE_X - 78, HOLE_Y - 83, Z2+4),

            # --- Cột C ---
            'C1': (HOLE_X - 55, HOLE_Y + 55, Z2),
            'C2': (HOLE_X - 50, HOLE_Y, Z2),
            'C3': (HOLE_X - 50, HOLE_Y - 50, Z2+4),

            # --- Cột D ---
            'D1': (HOLE_X - 22, HOLE_Y + 83, Z3),
            'D2': (HOLE_X - 27, HOLE_Y + 27, Z2),
            'D3': (HOLE_X - 25, HOLE_Y, Z1),
            'D4': (HOLE_X - 32, HOLE_Y - 27, Z2),
            'D5': (HOLE_X - 27, HOLE_Y - 78, Z3+3),

            # --- Cột E ---
            'E1': (HOLE_X + 5, HOLE_Y + 110, Z1),
            'E2': (HOLE_X + 5, HOLE_Y + 90,  Z1),
            'E3': (HOLE_X + 5, HOLE_Y + 70,  Z2),
            'E4': (HOLE_X + 5, HOLE_Y + 35,  Z1+1),
            'E5': (HOLE_X, HOLE_Y - 25,  Z1+3),
            'E6': (HOLE_X, HOLE_Y - 50,  Z2+5),
            'E7': (HOLE_X, HOLE_Y - 80,  Z1+5),
            'E8': (HOLE_X, HOLE_Y - 100, Z1+8),

            # --- Cột F ---
            'F1': (HOLE_X + 32, HOLE_Y + 83, Z3),
            'F2': (HOLE_X + 27, HOLE_Y + 37, Z2),
            'F3': (HOLE_X + 35, HOLE_Y + 5,  Z1+2),
            'F4': (HOLE_X + 27, HOLE_Y - 27, Z2+1),
            'F5': (HOLE_X + 25, HOLE_Y - 72, Z3+3),

            # --- Cột G ---
            'G1': (HOLE_X + 55, HOLE_Y + 70, Z2),
            'G2': (HOLE_X + 58, HOLE_Y + 10, Z2+4),
            'G3': (HOLE_X + 60, HOLE_Y - 60, Z2+6),

            # --- Cột H ---
            'H1': (HOLE_X + 80, HOLE_Y + 88, Z2+3),
            'H2': (HOLE_X + 83, HOLE_Y + 37, Z3+4),
            'H3': (HOLE_X + 80, HOLE_Y + 10, Z1+4), 
            'H4': (HOLE_X + 83, HOLE_Y - 12, Z3+3),
            'H5': (HOLE_X + 83, HOLE_Y - 68, Z2),

            # --- Cột I ---
            'I1': (HOLE_X + 105, HOLE_Y + 10, Z1+3),
        }

        
        self.setup_ui()
        # Binding phím x để thoát ứng dụng
        self.bind('x', lambda event: self.master.quit()) 
        self.focus_set() 
        self.protocol("WM_DELETE_WINDOW", self.on_closing)
        # Bắt đầu lắng nghe phản hồi từ RobotController
        self.after(100, self.check_logic_response)

    def setup_ui(self):
        main_frame = tk.Frame(self, bg="white", padx=10, pady=10)
        main_frame.pack(fill="both", expand=True)

        title_label = tk.Label(
            main_frame,
            text="BẢNG ĐIỀU KHIỂN",
            font=("Arial", 20, "bold"),
            bg="white",
            fg="black"
        )
        title_label.pack(pady=10)

        btn_frame = tk.Frame(main_frame, bg="white")
        btn_frame.pack(pady=10)
        
        commands = [
            ("HOME", "blue", "white", self.run_home_command),
            ("RUN", "green", "white", self.run_set_command),
            ("HOLE", "yellow", "black", self.run_hole_command),
            ("HOLE DW","orange","black",self.hole_up),
            ("RESET VẬT", "red", "black", self.reset_objects),
            ("AUTO", "pink", "black", self.auto_hut_objects),
        ]
        for text, bg, fg, cmd in commands:
            tk.Button(btn_frame, text=text, width=10, height=2, font=("Arial", 14, "bold"), 
                      bg=bg, fg=fg, command=cmd).pack(side=tk.LEFT, padx=10)

        # 2. KHU VỰC HIỂN THỊ TỌA ĐỘ VÀ JOINTS (Bên trái)
        content_frame = tk.Frame(main_frame, bg="white")
        content_frame.pack(fill="both", expand=True)
        
        left_panel = tk.Frame(content_frame, bg="white")
        left_panel.pack(side=tk.LEFT, fill="y", padx=10)

        # Bảng nhập tọa độ 5 vật
        inv_kin_frame = tk.LabelFrame(left_panel, text="Động học nghịch", font=("Arial", 12, "bold"), bg="white", padx=5, pady=5)
        inv_kin_frame.pack(fill="x")

        labels = ["Tên", "X", "Y", "Z"]
        for c, txt in enumerate(labels):
            tk.Label(inv_kin_frame, text=txt, font=("Arial", 10 , "bold"), bg="white").grid(row=0, column=c+1)

        for i in range(5):
            tk.Label(inv_kin_frame, text=f"Vật {i+1}", bg="white").grid(row=i+1, column=0)
            row_entries = []
            for j in range(4):
                e = tk.Entry(inv_kin_frame, width=8)
                e.grid(row=i+1, column=j+1, padx=2, pady=2)
                row_entries.append(e)
            self.all_object_entries.append(row_entries)

        # Bảng hiển thị Joint (Deg/Pulse)
        joint_frame = tk.LabelFrame(left_panel, text="Trạng thái Joint", font=("Arial", 11, "bold"), bg="white", padx=5, pady=5)
        joint_frame.pack(fill="x", pady=10)

        for i in range(3):
            tk.Label(joint_frame, text=f"Joint {i+1}", bg="white").grid(row=i, column=0)
            
            e_deg = tk.Entry(joint_frame, width=8, state='readonly')
            e_deg.grid(row=i, column=1, padx=5)
            self.entry_deg.append(e_deg)
            tk.Label(joint_frame, text="°", bg="white").grid(row=i, column=2)

            e_tar = tk.Entry(joint_frame, width=8, state='readonly')
            e_tar.grid(row=i, column=3, padx=5)
            self.entry_target.append(e_tar)
            tk.Label(joint_frame, text="Pulse", bg="white").grid(row=i, column=4)

        # 3. KHAY TỌA ĐỘ (Ở giữa)
        tray_frame = tk.LabelFrame(content_frame, text="Khay vật", font=("Arial", 12, "bold"), bg="white", padx=5, pady=5)
        tray_frame.pack(side=tk.LEFT, anchor="n", padx=5) # "n" là North - phía trên
  
        tray_grid = tk.Frame(tray_frame, bg="white")
        tray_grid.pack(expand=True)

        buttons = [
            ('E1',1,5), ('B1',2,1), ('D1',2,4), ('E2',2,5), ('F1',2,6), ('H1',2,9),
            ('C1',3,2), ('E3',3,5), ('G1',3,8), ('B2',4,1), ('D2',4,4), ('E4',4,5), ('F2',4,6), ('H2',4,9),
            ('A1',5,0), ('B3',5,1), ('C2',5,2), ('D3',5,4), ('F3',5,6), ('G2',5,8), ('H3',5,9), ('I1',5,10),
            ('B4',6,1), ('D4',6,4), ('E5',6,5), ('F4',6,6), ('H4',6,9), ('C3',7,2), ('E6',7,5), ('G3',7,8),
            ('B5',8,1), ('D5',8,4), ('E7',8,5), ('F5',8,6), ('H5',8,9), ('E8',9,5)
        ]
        for name, r, c in buttons:
            tk.Button(tray_grid, text=name, width=4, bg="lightgray",
                      command=lambda n=name: self.select_tray_coord(n)).grid(row=r, column=c, padx=5, pady=5)

        # 4. NAM CHÂM & THÔNG BÁO (Bên phải)
        right_panel = tk.Frame(content_frame, bg="white")
        right_panel.pack(side=tk.LEFT, fill="y", padx=10)

        mag_frame = tk.LabelFrame(right_panel, text="Nam châm", font=("Arial", 12, "bold"), bg="white", padx=10, pady=10)
        mag_frame.pack(fill="x")
        tk.Button(mag_frame, text="HÚT", width=7, height=1, bg="red", fg="white", font=("bold"), command=lambda: self.cmd_queue.put(("HUT", None))).pack(fill="x", pady=5)
        tk.Button(mag_frame, text="THẢ", width=7, height=1, bg="gray", fg="white", font=("bold"), command=lambda: self.cmd_queue.put(("THA", None))).pack(fill="x", pady=5)
        tk.Button(mag_frame, text="DOWN", width=7, height=1, bg="green", fg="white", font=("bold"), command=self.down).pack(fill="x", pady=5)
        tk.Button(mag_frame, text="UP", width=7, height=1, bg="blue", fg="white", font=("bold"), command=self.up).pack(fill="x", pady=5)
        
        noti_frame = tk.LabelFrame(left_panel, text="Thông báo", font=("Arial", 12, "bold"), bg="white", padx=10, pady=10)
        noti_frame.pack(fill="x", pady=10)
        self.notification_entry = tk.Entry(noti_frame, width=40, state="readonly", font=("Consolas", 10), fg="blue")
        self.notification_entry.pack(fill="x", ipady=20)

    # --- LOGIC XỬ LÝ ---
    def check_logic_response(self):
        try:
            msg_type, data = self.res_queue.get_nowait()
            if msg_type == "IK_SUCCESS":
                angles, pulses, idx = data
                self.update_joint_display(angles, pulses)
                if idx is not None: self.highlight_object(idx, "yellow")
                self.set_notification("Tính toán IK thành công. Đang di chuyển...")
            
            elif msg_type == "DONE":
                cmd, idx, angles = data
                self.set_notification(f"Hoàn thành lệnh: {cmd}")
                if cmd == "SET" and idx != -1:
                    self.highlight_object(idx, "lightgreen")
                elif cmd == "HOME":
                    self.update_joint_display([0,0,0], [0,0,0])
                # Nếu đang chạy auto thì DONE -> chạy bước tiếp theo
                if self.auto_running:
                    if self.auto_running:
                        delay = self.delay_map.get(cmd, 3000)
                        self.after(delay, self.run_next_auto)
                else:
                    # logic cũ (nếu bạn còn dùng execution_index cho chạy tay)
                    if cmd == "THA":
                        self.execution_index += 1
            elif msg_type == "ERROR":
                self.set_notification(f"LỖI: {data}")
                self.auto_running = False
            elif msg_type == "IK_FAILED":
                self.set_notification("Lỗi: Không tìm được nghiệm IK phù hợp!")
                self.auto_running = False
            
            elif msg_type == "LOG":
                self.set_notification(f"{data}")

        except queue.Empty: pass
        self.after(100, self.check_logic_response)

    def run_home_command(self):
        self.cmd_queue.put(("HOME", None))
        self.set_notification("Đang gửi lệnh HOME...")
    def down(self):      
        try:
            entries = self.all_object_entries[self.execution_index]
            px, py, pz = float(entries[1].get()), float(entries[2].get()), float(entries[3].get())
            self.cmd_queue.put(("SET", (px, py, pz, self.execution_index)))
        except: self.set_notification("Lỗi: Tọa độ không hợp lệ!")
    def up(self):      
        try:
            entries = self.all_object_entries[self.execution_index]
            px, py, pz = float(entries[1].get()), float(entries[2].get()), HOLE_Z
            self.cmd_queue.put(("SET", (px, py, pz, self.execution_index)))
        except: self.set_notification("Lỗi: Tọa độ không hợp lệ!")    
    def run_set_command(self):
        if self.execution_index >= self.input_index:
            self.set_notification("Chưa chọn đủ vật hoặc đã hoàn thành hết!")
            return
        try:
            entries = self.all_object_entries[self.execution_index]
            px, py, pz = float(entries[1].get()), float(entries[2].get()), HOLE_Z
            self.cmd_queue.put(("SET", (px, py, pz, self.execution_index)))
        except: self.set_notification("Lỗi: Tọa độ không hợp lệ!")

    def run_hole_command(self):
        self.cmd_queue.put(("HOLE", (HOLE_X, HOLE_Y, HOLE_Z, -1)))
    def hole_up(self):
        self.cmd_queue.put(("SET", (HOLE_X, HOLE_Y, HOLE_Z - 20, -1)))
    def select_tray_coord(self, name):
        if self.input_index >= 5: return
        x, y, z = self.tray_coordinates[name]
        vals = [name, x, y, z]
        for e, v in zip(self.all_object_entries[self.input_index], vals):
            e.delete(0, tk.END); e.insert(0, str(v))
        self.input_index += 1

    def update_joint_display(self, angles, pulses):
        for i in range(3):
            # Duyệt qua các cặp (ô nhập liệu, giá trị mới)
            for entry, val in [(self.entry_deg[i], f"{angles[i]:.2f}"), (self.entry_target[i], int(pulses[i]))]:
                entry.config(state='normal')   # 1. Mở khóa để cho phép chỉnh sửa
                entry.delete(0, tk.END)        # 2. Xóa toàn bộ nội dung từ vị trí 0 đến hết
                entry.insert(0, val)           # 3. Chèn giá trị mới vào vị trí đầu tiên
                entry.config(state='readonly') # 4. Khóa lại để người dùng không tự ý sửa

    def highlight_object(self, index, color):
        if 0 <= index < 5:
            self.all_object_entries[index][0].config(bg=color)

    def reset_objects(self):
            # 1. Xóa danh sách vật (tọa độ X, Y, Z...)
            for row in self.all_object_entries:
                for e in row:
                    e.config(state='normal') # Đảm bảo mở khóa để xóa nếu cần
                    e.delete(0, tk.END)
                    e.config(bg="white")
            
            # 2. Xóa các ô hiển thị Joints (Góc độ và Xung)
            for i in range(3):
                # Duyệt qua cả ô Deg và ô Target (Pulses)
                for entry in [self.entry_deg[i], self.entry_target[i]]:
                    entry.config(state='normal')   # Mở khóa
                    entry.delete(0, tk.END)        # Xóa nội dung
                    entry.config(state='readonly') # Khóa lại
            
            # 3. Reset các chỉ số và thông báo
            self.input_index = 0
            self.execution_index = 0
            self.set_notification("Đã reset danh sách vật và các khớp.")

    def set_notification(self, msg):
        self.notification_entry.config(state='normal')
        self.notification_entry.delete(0, tk.END); self.notification_entry.insert(0, msg)
        self.notification_entry.config(state='readonly')

    def on_closing(self): self.master.destroy()
    def auto_hut_objects(self):
        # Không cho chạy chồng
        if self.auto_running:
            self.set_notification("Đang chạy HÚT 5 rồi!")
            return

        # Cần có ít nhất 1 vật đã chọn
        if self.input_index == 0:
            self.set_notification("Chưa chọn vị trí nào trong khay!")
            return

        # Tạo danh sách lệnh (job)
        self.auto_job = []
        for i in range(self.input_index):  # thường là 5, nhưng cho linh hoạt
            try:
                entries = self.all_object_entries[i]
                x = float(entries[1].get())
                y = float(entries[2].get())
                z = float(entries[3].get())
            except:
                self.set_notification(f"Lỗi: Tọa độ vật {i+1} không hợp lệ!")
                self.auto_job = []
                return

            # --- Chuỗi thao tác cho 1 vật ---
            self.auto_job.append(("SET", (HOLE_X, HOLE_Y, HOLE_Z, i)))
            # 1) đến phía trên vật
            self.auto_job.append(("SET", (x, y, HOLE_Z, i)))
            # 2) hút
            self.auto_job.append(("HUT", None))
            # 3) hạ xuống đúng Z của vật
            self.auto_job.append(("SET", (x, y, z, i)))
            # 4) nâng lên
            self.auto_job.append(("SET", (x, y, HOLE_Z, i)))
            # 5) tới lỗ (trên)
            self.auto_job.append(("SET", (HOLE_X, HOLE_Y, HOLE_Z, -1)))
            # 6) hạ xuống lỗ (giống HOLE_DW)
            self.auto_job.append(("SET", (HOLE_X, HOLE_Y, HOLE_Z - 20, -1)))
            # 7) thả
            self.auto_job.append(("THA", None))
            # 8) nâng lên lỗ
            #self.auto_job.append(("SET", (HOLE_X, HOLE_Y, HOLE_Z, -1)))
            
        self.auto_running = True
        self.auto_step = 0
        self.set_notification(f"Bắt đầu vật {self.input_index}")
        self.run_next_auto()
    def run_next_auto(self):
        if not self.auto_running:
            return

        if self.auto_step >= len(self.auto_job):
            self.auto_running = False
            self.set_notification("Hoàn thành HÚT 5 tự động!")
            return

        cmd_type, data = self.auto_job[self.auto_step]
        self.auto_step += 1
        self.cmd_queue.put((cmd_type, data))
class StartPage(tk.Tk):
    def __init__(self, cmd_queue, res_queue):
        super().__init__()
        self.cmd_queue, self.res_queue = cmd_queue, res_queue
        self.title("BÁO CÁO CUỐI KỲ")
        self.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        # ---------------------------
        self.configure(bg="white")
        # Binding phím bất kỳ để thoát ứng dụng
        self.bind('x', lambda event: self.quit()) # Nhấn phím 'x' để thoát
        title = tk.Label(self, text="BÁO CÁO CUỐI KỲ", font=("Arial", 28, "bold"), fg="black", bg="white")
        title.pack(pady=5)
        subtitle = tk.Label(self, text="THỰC TẬP KỸ THUẬT ROBOT", font=("Arial", 28, "bold"), fg="black", bg="white")
        subtitle.pack(pady=5)
        robot = tk.Label(self, text="ROBOT DR3", font=("Arial", 28, "bold"), fg="red", bg="white")
        robot.pack(pady=5)
        frame = tk.Frame(self, bg="white", highlightbackground="black", highlightthickness=2)
        frame.pack(pady=30)
        group_title = tk.Label(frame, text="Thông tin nhóm:", font=("Arial", 16, "bold"), bg="white")
        group_title.grid(row=0, column=0, columnspan=2, pady=10, sticky="w", padx=20)
        labels = [
            ("GVHD:", "PGS.TS ĐẶNG XUÂN BA"),
            ("SVTH:", ""),
            ("NGUYỄN NGỌC HIỂN", "22151079"),
            ("HOÀNG THÁI BẢO", "22151051"),
            ("NGUYỄN THỊ TUYẾT MAI", "22151112"),
            ("PHẠM VĂN HOÀNG QUÂN", "22151142"),
        ]
        for i, (left, right) in enumerate(labels, start=1):
            lbl_left = tk.Label(frame, text=left, font=("Arial", 14, "bold"), bg="white")
            lbl_left.grid(row=i, column=0, sticky="w", padx=20, pady=6)
            lbl_right = tk.Label(frame, text=right, font=("Arial", 14, "bold"), bg="white")
            lbl_right.grid(row=i, column=1, sticky="w", padx=40, pady=6)
        self.start_button = tk.Button(self, text="START", font=("Arial", 18, "bold"),
                                      bg="#4CAF50", fg="white", padx=20, pady=8,
                                      command=self.open_control)
        self.start_button.pack(pady=10)

    def open_control(self):
        self.withdraw()
        ControlPanel(self, self.cmd_queue, self.res_queue)