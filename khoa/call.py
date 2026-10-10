import requests
import time
import os
from datetime import datetime
from ultralytics import YOLO

# ===== Cấu hình =====
ESP32_IP = "10.70.96.171"  # IP in ra ở Serial Monitor

SAVE_DIR = r"C:\Users\AD\Downloads\yolo\cho\real\anh_esp"   # nơi lưu ảnh gốc
RESULT_DIR = r"C:\Users\AD\Downloads\yolo\cho\real\ket_qua" # nơi lưu kết quả YOLO
MODEL_PATH = r"C:\Users\AD\Downloads\yolo\cho\real\best.pt"  # đường dẫn tới file best.pt, đổi nếu để chỗ khác

CAPTURE_INTERVAL = 15  # giây

os.makedirs(SAVE_DIR, exist_ok=True)
os.makedirs(RESULT_DIR, exist_ok=True)

print("Ảnh gốc lưu tại:", os.path.abspath(SAVE_DIR))
print("Kết quả YOLO lưu tại:", os.path.abspath(RESULT_DIR))

# Load model 1 lần duy nhất (không load lại mỗi vòng lặp, tốn thời gian)
print("Đang load model...")
model = YOLO(MODEL_PATH)
print("Model đã sẵn sàng!")

while True:
    try:
        r = requests.get(f"http://{ESP32_IP}/capture", timeout=5)
        if r.status_code == 200:
            filename = datetime.now().strftime("%Y%m%d_%H%M%S") + ".jpg"
            img_path = os.path.join(SAVE_DIR, filename)

            with open(img_path, "wb") as f:
                f.write(r.content)
            print(f"Đã lưu ảnh: {filename}")

            # Chạy YOLO ngay trên ảnh vừa chụp
            results = model.predict(
                source=img_path,
                save=True,
                save_txt=True,
                project=RESULT_DIR,
                name="predict",
                exist_ok=True,  # ghi tiếp vào cùng 1 folder predict, không tạo predict2, predict3...
                verbose=False,  # tắt log dài dòng của YOLO cho mỗi ảnh
            )

            # In nhanh số object phát hiện được
            num_detections = len(results[0].boxes)
            print(f"  -> Phát hiện {num_detections} đối tượng")

        else:
            print(f"Lỗi HTTP: {r.status_code}")

    except Exception as e:
        print("Error:", e)

    time.sleep(CAPTURE_INTERVAL)