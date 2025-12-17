import can
import time
import os
import cv2
import pandas as pd
from pypylon import pylon
from ultralytics import YOLO

# --------------------------- Settings ---------------------------
BASE_SAVE_DIR = r"C:\Users\hasib\OneDrive\Radna površina\TESTING"
os.makedirs(BASE_SAVE_DIR, exist_ok=True)

CAPTURE_FPS = 5
CAPTURE_INTERVAL = 1.0 / CAPTURE_FPS

CAN_TIMEOUT = 0.005  # 🔑 VERY IMPORTANT

# --------------------------- Load YOLO ---------------------------
print("🚀 Loading YOLO model...")
model = YOLO(r"C:\yolo_models\yolov10n.pt")

# --------------------------- Start Basler camera ---------------------------
print("📡 Starting Basler camera...")
devices = list(pylon.TlFactory.GetInstance().EnumerateDevices())
if not devices:
    raise SystemExit("❌ No Basler camera found!")

camera = pylon.InstantCamera(
    pylon.TlFactory.GetInstance().CreateDevice(devices[0])
)
camera.Open()
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

# Warm-up
for _ in range(10):
    grab = camera.RetrieveResult(100, pylon.TimeoutHandling_Return)
    if grab and grab.GrabSucceeded():
        grab.Release()

print("🔥 Camera ready")

# --------------------------- Setup CAN ---------------------------
bus = can.Bus(
    interface="vector",
    channel=1,
    bitrate=500000,
    app_name=""
)
print("✅ Connected to CAN channel")

target_id = 0x7B
prev_value = -1

# --------------------------- Session State ---------------------------
capturing = False
session_dir = None
excel_path = None
records = []
img_counter = 0
last_capture_time = 0.0

# --------------------------- Helper functions ---------------------------
def start_session():
    global capturing, session_dir, excel_path
    global records, img_counter, last_capture_time

    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    session_dir = os.path.join(BASE_SAVE_DIR, f"session_{timestamp}")
    os.makedirs(session_dir, exist_ok=True)

    excel_path = os.path.join(session_dir, "detections.xlsx")

    records.clear()
    img_counter = 0
    last_capture_time = 0.0
    capturing = True

    print(f"📁 New session started: {session_dir}")

def stop_session():
    global capturing

    capturing = False

    if records:
        pd.DataFrame(records).to_excel(excel_path, index=False)
        print(f"📊 Excel saved: {excel_path}")

    print("🛑 Session stopped, waiting for next trigger")

# --------------------------- Main Loop ---------------------------
try:
    while True:
        # ==========================================================
        # 1️⃣ CAN ALWAYS FIRST (REAL-TIME SAFE)
        # ==========================================================
        msg = bus.recv(timeout=CAN_TIMEOUT)

        if msg and msg.arbitration_id == target_id and msg.data:
            signal = msg.data[0]

            if signal != prev_value:
                print(f"🔄 Signal changed: {signal}")
                prev_value = signal

            if signal == 2 and not capturing:
                start_session()

            elif signal == 1 and capturing:
                stop_session()

        # ==========================================================
        # 2️⃣ CAMERA + YOLO (TIME-CONTROLLED)
        # ==========================================================
        now = time.time()
        if capturing and (now - last_capture_time) >= CAPTURE_INTERVAL:
            last_capture_time = now

            grab = camera.RetrieveResult(
                100, pylon.TimeoutHandling_Return
            )
            if not grab.GrabSucceeded():
                grab.Release()
                continue

            frame = converter.Convert(grab).GetArray()
            grab.Release()

            img_counter += 1
            img_name = f"img_{img_counter:04d}.jpg"
            img_path = os.path.join(session_dir, img_name)

            # ✅ Save RAW image
            cv2.imwrite(img_path, frame)

            # YOLO (blocking is OK here)
            results = model(frame, conf=0.5, verbose=False)

            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])

                # ❌ Ignore detections above y = 230
                if y2 < 230:
                    continue

                cls = int(box.cls)

                records.append({
                    "Image File": img_name,
                    "Class": results[0].names[cls],
                    "Confidence": float(box.conf[0]),
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2
                })

except KeyboardInterrupt:
    print("\n🛑 Ctrl+C pressed")

finally:
    if capturing and records and excel_path:
        pd.DataFrame(records).to_excel(excel_path, index=False)
        print(f"📊 Excel saved: {excel_path}")

    camera.StopGrabbing()
    camera.Close()
    cv2.destroyAllWindows()
    bus.shutdown()

    print("✅ System shutdown complete")
