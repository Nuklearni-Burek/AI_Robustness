# fast_can_yolo.py
import can
import time, os, cv2, pandas as pd
from pypylon import pylon
from ultralytics import YOLO

# --------------------------- Settings ---------------------------
save_dir = r"C:\Users\hasib\OneDrive\Radna površina\TESTING"
excel_path = os.path.join(save_dir, "detections.xlsx")
os.makedirs(save_dir, exist_ok=True)

# --------------------------- Load YOLO ---------------------------
print("🚀 Loading YOLO model...")
model = YOLO(r"C:\yolo_models\yolov10n.pt")

# --------------------------- Start Basler camera ---------------------------
print("📡 Starting Basler camera...")
devices = list(pylon.TlFactory.GetInstance().EnumerateDevices())
if not devices:
    raise SystemExit("❌ No Basler camera found!")

camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[0]))
camera.Open()
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

# Warmup frames
for _ in range(10):
    grab = camera.RetrieveResult(100, pylon.TimeoutHandling_Return)
    if grab and grab.GrabSucceeded():
        grab.Release()

print("🔥 Camera ready. Monitoring CAN signals...")

# --------------------------- Setup CAN ---------------------------
bus = can.Bus(interface="vector", channel=1, bitrate=500000, app_name="")
print("✅ Connected to CAN channel 1")

target_id = 0x7b
prev_value = -1
records = []

# --------------------------- Main Loop ---------------------------
try:
    start_time = time.time()
    while True:
        msg = bus.recv(timeout=0.05)
        if msg and msg.arbitration_id == target_id and msg.data:
            current_value = msg.data[0]
            if current_value != prev_value:
                elapsed = time.time() - start_time
                print(f"[{elapsed:7.3f}s] 🔄 Signal: {current_value}")
                prev_value = current_value

                if current_value == 2:
                    print("📸 Trigger signal detected → Capturing image")

                    # Capture latest image
                    grab = camera.RetrieveResult(100, pylon.TimeoutHandling_Return)
                    if grab.GrabSucceeded():
                        frame = converter.Convert(grab).GetArray()
                    grab.Release()

                    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
                    img_filename = f"capture_{timestamp}.jpg"
                    img_path = os.path.join(save_dir, img_filename)

                    # YOLO inference
                    results = model(frame, conf=0.5, verbose=False)
                    cv2.imwrite(img_path, results[0].plot())
                    print(f"💾 Image saved: {img_path}")

                    # Store detections in memory
                    for box in results[0].boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cls = int(box.cls)
                        records.append({
                            "Timestamp": timestamp,
                            "Image File": img_filename,
                            "Class": results[0].names[cls],
                            "Confidence": float(box.conf[0]),
                            "x1": x1, "y1": y1, "x2": x2, "y2": y2
                        })

except KeyboardInterrupt:
    print("\n🛑 Ctrl+C pressed → shutting down")

finally:
    # Save all detections to Excel
    if records:
        df = pd.DataFrame(records)
        if os.path.exists(excel_path):
            df_existing = pd.read_excel(excel_path)
            df = pd.concat([df_existing, df], ignore_index=True)
        df.to_excel(excel_path, index=False)
        print(f"📊 Excel updated: {excel_path}")

    camera.StopGrabbing()
    camera.Close()
    cv2.destroyAllWindows()
    bus.shutdown()
    print("✅ System shutdown complete")
