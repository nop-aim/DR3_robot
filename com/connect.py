import serial
import time
import threading
import queue

# ================================
# Cấu hình
# ================================
SERIAL_PORT = "COM6"
BAUDRATE = 115200

ser = None
serial_lock = threading.Lock()

# Hàng đợi chứa dữ liệu Arduino gửi về
serial_queue = queue.Queue()


# ======================================================
# THREAD ĐỌC SERIAL NỀN — KHÔNG BAO GIỜ DỪNG
# CHỈ ĐƯA DỮ LIỆU VÀO QUEUE MỘT CÁCH IM LẶNG
# ======================================================
def serial_reader_thread():
    buffer = ""   # bộ đệm để ghép dòng
    while True:
        if ser is None:
            continue
            
        try:
            data = ser.read(64).decode(errors='ignore')  # đọc chunk
            if not data:
                continue

            buffer += data

            # Tách thành các dòng hoàn chỉnh
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.replace("\r", "").strip()
                if line != "":
                    serial_queue.put(line)

        except Exception as e:
            print("Serial thread error:", e)

        time.sleep(0.005)   # giảm tải CPU


# ======================================================
# MỞ SERIAL + KHỞI ĐỘNG THREAD ĐỌC NỀN
# ======================================================
def open_serial():
    global ser

    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.01)
        time.sleep(2)
        print("Serial đã mở:", SERIAL_PORT)
    except Exception as e:
        ser = None
        print("Không mở được serial:", e)
        raise e

    # Tạo thread nền
    reader = threading.Thread(target=serial_reader_thread, daemon=True)
    reader.start()
    print("Thread đọc Serial đã chạy (chỉ đưa dữ liệu vào queue).")


# ======================================================
# CHỈ GỬI — KHÔNG ĐỌC Ở ĐÂY
# ======================================================
def send_serial_line(line):
    with serial_lock:
        if ser and ser.is_open:
            try:
                ser.write((line + "\n").encode('utf-8'))
                print("[SEND]:", line)
            except Exception as e:
                print("Lỗi gửi serial:", e)
        else:
            print("Serial chưa mở")


# ======================================================
# GỬI LỆNH SET GÓC (GIỮ NGUYÊN)
# ======================================================
def send_set_angles_cmd(t1, t2, t3):
    cmd = f"SET {t1:.2f} {t2:.2f} {t3:.2f}"
    send_serial_line(cmd)


# Gửi lệnh HOME (GIỮ NGUYÊN)
def send_home_cmd():
    send_serial_line("HOME")
def send_hut_cmd():
    send_serial_line("HUT")
def send_tha_cmd():
    send_serial_line("THA")

# ======================================================
# CHỜ NHẬN DONE TỪ ARDUINO —— KHÔNG BAO GIỜ MẤT DỮ LIỆU
# ======================================================
def wait_for_pid_done(timeout=None):
    """
    Đọc dữ liệu từ queue và kiểm tra DONE.
    Hàm này không in log; logic in log sẽ nằm trong main.py
    """
    while True:
        # Lấy dữ liệu từ queue nếu có
        try:
            msg = serial_queue.get_nowait()
            if msg == "DONE" or msg == "ALL DONE":
                return True
        except queue.Empty:
            pass

        time.sleep(0.01)
def wait_for_hut_done(timeout=None):
    """
    Đọc dữ liệu từ queue và kiểm tra DONE.
    Hàm này không in log; logic in log sẽ nằm trong main.py
    """
    while True:
        # Lấy dữ liệu từ queue nếu có
        try:
            msg = serial_queue.get_nowait()
            if msg == "HUT DONE":
                return True
        except queue.Empty:
            pass

        time.sleep(0.01)
def wait_for_tha_done(timeout=None):
    """
    Đọc dữ liệu từ queue và kiểm tra DONE.
    Hàm này không in log; logic in log sẽ nằm trong main.py
    """
    while True:
        # Lấy dữ liệu từ queue nếu có
        try:
            msg = serial_queue.get_nowait()
            if msg == "THA DONE":
                return True
        except queue.Empty:
            pass

        time.sleep(0.01)
def wait_for_home_done(timeout=None):
    """
    Đọc dữ liệu từ queue và kiểm tra DONE.
    Hàm này không in log; logic in log sẽ nằm trong main.py
    """
    while True:
        # Lấy dữ liệu từ queue nếu có
        try:
            msg = serial_queue.get_nowait()
            if msg == "HOME DONE":
                return True
        except queue.Empty:
            pass

        time.sleep(0.01)
        