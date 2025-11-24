from ultralytics import YOLO
from pypylon import pylon
import cv2

# ---------------------------
# 1. YOLO Setup
# ---------------------------
model = YOLO("yolov10n.pt")   # You can change to yolov10s.pt for better accuracy

# ---------------------------
# 2. Basler Camera Setup
# ---------------------------
devices = list(pylon.TlFactory.GetInstance().EnumerateDevices())
if not devices:
    raise SystemExit("❌ No Basler camera found!")

camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[0]))
camera.Open()
camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

print("🚀 YOLO detection active. Press 'q' to quit.")

# ---------------------------
# 3. Main Loop
# ---------------------------
try:
    while camera.IsGrabbing():
        grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if not grabResult.GrabSucceeded():
            continue

        image = converter.Convert(grabResult)
        frame = image.GetArray()
        grabResult.Release()

        # Run YOLO
        results = model.predict(frame, imgsz=640, conf=0.5, verbose=False)

        # Extract detections
        boxes = []
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            width = x2 - x1
            height = y2 - y1
            cls_id = int(box.cls)
            label = results[0].names[cls_id]
            confidence = float(box.conf[0])
            boxes.append((x1, y1, width, height, label, confidence))

        # Draw boxes
        annotated = frame.copy()
        for x1, y1, w, h, label, conf in boxes:
            cv2.rectangle(annotated, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)
            cv2.putText(annotated, f"{label} {conf:.2f}", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Show video
        cv2.imshow("Basler Object Detection", cv2.resize(annotated, (0, 0), fx=0.5, fy=0.5))

        # Print detections
        if boxes:
            print("🔹 Detected objects:")
            for x1, y1, w, h, label, conf in boxes:
                print(f"  - {label} | Conf: {conf:.2f} | BBox: ({x1}, {y1}, {x1+w}, {y1+h})")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("🛑 Quitting...")
            break

finally:
    camera.StopGrabbing()
    camera.Close()
    cv2.destroyAllWindows()
    print("✅ Camera closed and program ended cleanly.")
