
import cv2
import time
import traceback
from pathlib import Path
from ultralytics import YOLO

cap = None

try:
    # Lấy đường dẫn best.pt cùng thư mục với file Python
    model_path = Path(__file__).resolve().parent / "best.pt"

    if not model_path.exists():
        raise FileNotFoundError(
            f"Khong tim thay model: {model_path}"
        )

    print("Dang tai model YOLO...")
    model = YOLO(str(model_path))
    print("Tai model thanh cong!")

    # Mở webcam
    print("Dang mo webcam...")
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        raise RuntimeError(
            "Khong mo duoc webcam. Hay kiem tra camera."
        )

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    print("Webcam da mo. Nhan Q de thoat.")

    prev_time = time.time()

    while True:
        ret, frame = cap.read()

        if not ret:
            print("Khong doc duoc khung hinh tu webcam!")
            break

        results = model.predict(
            source=frame,
            imgsz=640,
            conf=0.5,
            verbose=False
        )

        output = results[0].plot()

        # Tinh FPS
        now = time.time()
        fps = 1.0 / max(now - prev_time, 1e-6)
        prev_time = now

        cv2.putText(
            output,
            f"FPS: {fps:.1f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        cv2.imshow("YOLO Webcam", output)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break

except Exception:
    print("\nDA XAY RA LOI:")
    traceback.print_exc()

finally:
    if cap is not None:
        cap.release()

    cv2.destroyAllWindows()

    input("\nNhan Enter de dong chuong trinh...")

