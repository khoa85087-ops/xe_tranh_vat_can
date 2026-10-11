import socket
import struct
import time
from typing import Optional

import cv2
import numpy as np
import requests
from ultralytics import YOLO


# ============================================================
# SETTINGS
# ============================================================
ESP32_IP = "192.168.31.51"
CAMERA_PORT = 3333
COMMAND_URL = f"http://{ESP32_IP}/command"

MAX_JPEG_SIZE = 1_000_000
YOLO_MODEL = "yolo26n.pt"
YOLO_CONFIDENCE = 0.5
YOLO_IMAGE_SIZE = 320

# Repeat current command at 5 Hz while testing.
# The supplied STM32 main.c has no command-timeout failsafe,
# so keep a supervised test setup and wheels lifted.
COMMAND_SEND_INTERVAL = 0.2

DISPLAY_WIDTH = 960
DISPLAY_HEIGHT = 720

# This is a demonstration policy, not a finished navigation algorithm.
# AUTO mode:
#   no person / person centered -> STOP
#   person left -> left steering, strength based on offset
#   person right -> right steering, strength based on offset
#
# MANUAL mode keys:
#   w = FORWARD
#   2/3/4 = RIGHT_SMALL / RIGHT_MEDIUM / RIGHT_LARGE
#   5/6/7 = LEFT_SMALL / LEFT_MEDIUM / LEFT_LARGE
#   s = BACKWARD
#   x = STOP
#   m = switch AUTO/MANUAL
#   q = send STOP and quit


# ============================================================
# CAMERA TCP HELPERS
# ============================================================
def read_exact(sock: socket.socket, length: int) -> Optional[bytes]:
    data = bytearray()
    while len(data) < length:
        try:
            packet = sock.recv(length - len(data))
        except socket.timeout:
            return None
        except OSError:
            return None

        if not packet:
            return None

        data.extend(packet)
    return bytes(data)


def find_header(sock: socket.socket) -> bool:
    buffer = bytearray()
    while True:
        byte = read_exact(sock, 1)
        if byte is None:
            return False

        buffer.extend(byte)
        if len(buffer) > 4:
            del buffer[:-4]

        if bytes(buffer) == b"FRAM":
            return True


def receive_jpeg_frame(sock: socket.socket) -> Optional[bytes]:
    if not find_header(sock):
        return None

    size_bytes = read_exact(sock, 4)
    if size_bytes is None:
        return None

    jpeg_size = struct.unpack("<I", size_bytes)[0]
    if jpeg_size <= 0 or jpeg_size > MAX_JPEG_SIZE:
        print(f"[CAMERA] Invalid JPEG size: {jpeg_size}")
        return None

    return read_exact(sock, jpeg_size)


# ============================================================
# COMMAND MAPPING / HTTP JSON
# Names here are translated by the ESP32 into the single-byte
# codes used by main.c on the STM32.
# ============================================================
COMMAND_TO_STM32_CODE = {
    "FORWARD": "1",
    "RIGHT_SMALL": "2",
    "RIGHT_MEDIUM": "3",
    "RIGHT_LARGE": "4",
    "LEFT_SMALL": "5",
    "LEFT_MEDIUM": "6",
    "LEFT_LARGE": "7",
    "BACKWARD": "8",
    "STOP": "0",
}


def send_command(
    command: str,
    source: str,
    detection: Optional[dict] = None,
) -> bool:
    if command not in COMMAND_TO_STM32_CODE:
        print(f"[COMMAND] Refusing unknown command: {command}")
        return False

    payload = {
        "type": "command",
        "command": command,
        "source": source,
    }

    if detection is not None:
        payload.update({
            "object": detection["object"],
            "confidence": round(float(detection["confidence"]), 2),
            "position": detection["position"],
            "center_x": int(detection["center_x"]),
        })

    try:
        response = requests.post(COMMAND_URL, json=payload, timeout=1.0)
        if response.status_code != 200:
            print(
                f"[HTTP ERROR] {response.status_code}: {response.text}"
            )
            return False

        try:
            result = response.json()
        except ValueError:
            print("[HTTP ERROR] ESP32 response was not JSON.")
            return False

        if result.get("status") != "OK":
            print(f"[HTTP ERROR] ESP32 rejected command: {result}")
            return False

        # Response means the ESP32 accepted/wrote the UART byte.
        # It does not prove the STM32 has executed the motor command.
        return True

    except requests.RequestException as exc:
        print(f"[HTTP ERROR] Could not send {command}: {exc}")
        return False


# ============================================================
# YOLO -> MOTOR COMMAND
# Uses the detected person's horizontal offset to choose the
# turn direction and intensity. No detection or centered person
# produces STOP as a conservative demo policy.
# ============================================================
def choose_auto_command(detections: list, frame_width: int):
    people = [item for item in detections if item["object"] == "person"]

    if not people:
        return "STOP", None

    target = max(people, key=lambda item: item["confidence"])
    offset = (target["center_x"] - frame_width / 2) / (frame_width / 2)
    abs_offset = abs(offset)

    # Central region: do not continue blindly.
    if abs_offset <= 0.15:
        return "STOP", target

    if abs_offset <= 0.35:
        level = "SMALL"
    elif abs_offset <= 0.65:
        level = "MEDIUM"
    else:
        level = "LARGE"

    # Match the earlier demo policy: turn toward the side where the
    # detected person appears. For obstacle avoidance, reverse this
    # direction after validating the robot's intended behavior.
    if offset < 0:
        return f"LEFT_{level}", target
    return f"RIGHT_{level}", target


# ============================================================
# MAIN
# ============================================================
def main():
    print("Loading YOLO model...")
    model = YOLO(YOLO_MODEL)
    print("YOLO loaded.")

    camera_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    camera_sock.settimeout(5)

    try:
        print(f"Connecting to camera {ESP32_IP}:{CAMERA_PORT}...")
        camera_sock.connect((ESP32_IP, CAMERA_PORT))
        print("Camera TCP connected.")
    except OSError as exc:
        camera_sock.close()
        raise SystemExit(
            f"Could not connect to camera: {exc}\n"
            "Check that ESP32-CAM is running and laptop is on the same Wi-Fi."
        )

    mode = "MANUAL"  # start safely; press M to switch to AUTO
    manual_command = "STOP"
    last_sent_command = None
    last_send_time = 0.0
    last_log_time = 0.0

    cv2.namedWindow("ESP32-CAM + YOLO / HTTP JSON", cv2.WINDOW_NORMAL)
    cv2.resizeWindow(
        "ESP32-CAM + YOLO / HTTP JSON",
        DISPLAY_WIDTH,
        DISPLAY_HEIGHT,
    )

    print("\nManual controls (start in MANUAL mode):")
    print("  W: forward ('1')")
    print("  2/3/4: right small/medium/large")
    print("  5/6/7: left small/medium/large")
    print("  S: backward ('8')")
    print("  X: stop ('0')")
    print("  M: toggle AUTO/MANUAL")
    print("  Q: stop and quit\n")
    print("HTTP command endpoint:", COMMAND_URL)

    try:
        while True:
            jpg = receive_jpeg_frame(camera_sock)
            if jpg is None:
                print("[CAMERA] Frame receive failed; stopping.")
                send_command("STOP", source="SAFETY")
                break

            frame = cv2.imdecode(
                np.frombuffer(jpg, dtype=np.uint8),
                cv2.IMREAD_COLOR,
            )
            if frame is None:
                print("[CAMERA] JPEG decode failed.")
                continue

            height, width = frame.shape[:2]
            results = model(
                frame,
                imgsz=YOLO_IMAGE_SIZE,
                conf=YOLO_CONFIDENCE,
                verbose=False,
            )
            result = results[0]
            detections = []

            for box in result.boxes:
                class_id = int(box.cls[0])
                object_name = model.names[class_id]
                confidence = float(box.conf[0])
                if confidence < YOLO_CONFIDENCE:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2

                if center_x < width / 3:
                    position = "left"
                elif center_x < 2 * width / 3:
                    position = "center"
                else:
                    position = "right"

                detection = {
                    "object": object_name,
                    "confidence": confidence,
                    "position": position,
                    "center_x": center_x,
                    "center_y": center_y,
                }
                detections.append(detection)

                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
                cv2.putText(
                    frame,
                    f"{object_name} {confidence:.2f}",
                    (x1, max(y1 - 8, 18)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    2,
                )
                cv2.circle(frame, (center_x, center_y), 3, (255, 255, 255), -1)

            auto_command, target = choose_auto_command(detections, width)

            display_command = manual_command if mode == "MANUAL" else auto_command
            display_frame = cv2.resize(
                frame,
                (DISPLAY_WIDTH, DISPLAY_HEIGHT),
                interpolation=cv2.INTER_LINEAR,
            )

            cv2.putText(
                display_frame,
                f"MODE: {mode}   COMMAND: {display_command}",
                (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
            )

            cv2.imshow("ESP32-CAM + YOLO / HTTP JSON", display_frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                send_command("STOP", source="SAFETY")
                break

            if key == ord("m"):
                mode = "AUTO" if mode == "MANUAL" else "MANUAL"
                if mode == "MANUAL":
                    manual_command = "STOP"
                print(f"[MODE] {mode}")

            if mode == "MANUAL":
                manual_keys = {
                    ord("w"): "FORWARD",
                    ord("2"): "RIGHT_SMALL",
                    ord("3"): "RIGHT_MEDIUM",
                    ord("4"): "RIGHT_LARGE",
                    ord("5"): "LEFT_SMALL",
                    ord("6"): "LEFT_MEDIUM",
                    ord("7"): "LEFT_LARGE",
                    ord("s"): "BACKWARD",
                    ord("x"): "STOP",
                }
                if key in manual_keys:
                    manual_command = manual_keys[key]
                    print(f"[MANUAL] {manual_command}")

            current_time = time.time()
            if current_time - last_send_time >= COMMAND_SEND_INTERVAL:
                command = manual_command if mode == "MANUAL" else auto_command
                command_source = "MANUAL" if mode == "MANUAL" else "YOLO"
                detection_for_payload = (
                    target if mode == "AUTO" and target is not None else None
                )

                ok = send_command(
                    command=command,
                    source=command_source,
                    detection=detection_for_payload,
                )

                # Log on command transitions (or failures) to reduce console spam.
                if command != last_sent_command or not ok:
                    if ok:
                        print(
                            f"[HTTP JSON] {command} -> "
                            f"STM32 byte '{COMMAND_TO_STM32_CODE[command]}' "
                            f"(ESP32 accepted UART write)"
                        )
                    last_log_time = current_time

                last_sent_command = command if ok else last_sent_command
                last_send_time = current_time

    except KeyboardInterrupt:
        print("Stopped by user.")
        send_command("STOP", source="SAFETY")
    finally:
        camera_sock.close()
        cv2.destroyAllWindows()
        print("Closed.")


if __name__ == "__main__":
    main()
