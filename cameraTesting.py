import paho.mqtt.client as mqtt
import json
from ultralytics import YOLO
from pypylon import pylon
import cv2
import threading

# ---------------------------
# 1. MQTT Setup
# ---------------------------
BROKER = "fmendt.site"
PORT = 8883
TOPIC = "control/ligth"  # Your sensor topic
USERNAME = "mechlab"
PASSWORD = "Hh9yUz4N!A"

LS1 = "00"  # default sensor state

def on_message(client, userdata, msg):
    """Handles incoming MQTT messages."""
    global LS1
    try:
        payload = msg.payload.decode()
        data = json.loads(payload)

        # Handle JSON payload like {"LS1": "01"}
        if isinstance(data, dict):
            LS1 = str(data.get("LS1", LS1))
        else:
            # Handle non-JSON numeric/string payload like "01"
            LS1 = str(data)

    except json.JSONDecodeError:
        LS1 = msg.payload.decode()
        print("Received non-JSON data:", LS1)


# MQTT client setup
client = mqtt.Client(protocol=mqtt.MQTTv311)
client.username_pw_set(USERNAME, PASSWORD)
client.tls_set()
client.on_message = on_message
client.connect(BROKER, PORT, keepalive=60)
client.subscribe(TOPIC)

# Run MQTT loop in a separate thread
mqtt_thread = threading.Thread(target=client.loop_forever)
mqtt_thread.daemon = True
mqtt_thread.start()

print(f"📡 Connected to broker {BROKER}, listening for sensor data...")

# ---------------------------
# 2. YOLO Setup
# ---------------------------
model = YOLO("yolov10n.pt")  # You can also try yolov10s.pt for more accuracy

# ---------------------------
# 3. Basler Camera Setup
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

print("🚀 YOLO detection active. Will process frames only when LS1 = '01'. Press 'q' to quit.")

# ---------------------------
# 4. Main Loop
# ---------------------------
try:
    while camera.IsGrabbing():
        # Always grab a frame (prevents timeout)
        grabResult = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
        if not grabResult.GrabSucceeded():
            continue

        image = converter.Convert(grabResult)
        frame = image.GetArray()
        grabResult.Release()

        # If LS1 != "01", skip YOLO detection
        if LS1 != "01":
            cv2.imshow("Camera Feed", cv2.resize(frame, (0, 0), fx=0.5, fy=0.5))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("🛑 Quitting...")
                break
            continue

        # YOLO detection when LS1 == "01"
        results = model.predict(frame, imgsz=640, conf=0.5, verbose=False)
        boxes = []
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            width = x2 - x1
            height = y2 - y1
            cls_id = int(box.cls)
            label = results[0].names[cls_id]
            confidence = float(box.conf[0])
            boxes.append((x1, y1, width, height, label, confidence))

        # Draw bounding boxes
        annotated = frame.copy()
        for x1, y1, w, h, label, conf in boxes:
            cv2.rectangle(annotated, (x1, y1), (x1 + w, y1 + h), (0, 255, 0), 2)
            cv2.putText(annotated, f"{label} {conf:.2f}", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow("Object Detection (LS1 = 01)", cv2.resize(annotated, (0, 0), fx=0.5, fy=0.5))

        # Log detected objects to terminal
        if boxes:
            print("🔹 Detected objects (LS1 = '01'):")
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
