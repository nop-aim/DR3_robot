import threading
import queue
import sys
import tkinter as tk

# Import từ các package theo cấu trúc thư mục của bạn
from gui.control_panel import StartPage
from robot_controller.robot_controller import RobotController

def main():
    # 1. Tạo 2 đường ống giao tiếp (Queue)
    gui_to_robot = queue.Queue()
    robot_to_gui = queue.Queue()

    # 2. Khởi tạo Controller và chạy trong Thread riêng
    controller = RobotController(gui_to_robot, robot_to_gui)
    logic_thread = threading.Thread(target=controller.run, daemon=True)
    logic_thread.start()

    # 3. Khởi tạo Giao diện (StartPage)
    # Truyền Queue vào để từ StartPage có thể truyền xuống ControlPanel
    app = StartPage(gui_to_robot, robot_to_gui)
    
    print("Hệ thống Robot DR3 đã sẵn sàng.")
    
    try:
        app.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        gui_to_robot.put(("QUIT", None))
        sys.exit()

if __name__ == "__main__":
    main()