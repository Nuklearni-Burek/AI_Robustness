# 🚗 CAN-Triggered Real-Time Vehicle Detection for Robustness Evaluation

## Abstract

This project presents a real-time vehicle detection framework developed for robustness evaluation of YOLO-based object detection models in controlled automotive scenarios. The system integrates industrial image acquisition with CAN-based session triggering to enable structured and repeatable measurement procedures.

The current implementation is based on **YOLOv10n**, with planned comparative evaluation using **YOLOv26**.

---

## Experimental Setup

### Image Acquisition

Images are acquired using a Basler industrial camera via the `pypylon` SDK.
The camera operates at **5 FPS** and records raw images exclusively during CAN-triggered measurement sessions.

### Session Control (CAN Bus)

Measurement sessions are controlled via CAN communication:

* **Arbitration ID:** `0x7B`
* **Signal `2` → Start session**
* **Signal `1` → Stop session**

This ensures deterministic and reproducible recording conditions similar to real automotive environments.

### Object Detection

Vehicle detection is performed using the Ultralytics implementation of **YOLOv10n** with a confidence threshold of **0.5**.

For each detection, the following parameters are stored:

* Class label
* Confidence score
* Bounding box coordinates

All results are exported to **structured Excel files** for statistical robustness analysis.

---

## Test Vehicles

Robustness evaluation was conducted using six vehicles with diverse geometrical characteristics:

* Mercedes EQE
* Škoda Enyaq
* Mazda MX-5
* Seat Cordoba
* Opel Movano
* Mercedes-Benz G-Klasse (Puch variant)

Repeated measurements were performed to analyze:

* Confidence stability
* Detection consistency
* Class robustness across varying vehicle shapes and sizes

---

## Demonstration Videos

[![Session 1](https://img.youtube.com/vi/U2rkALwhL6I/maxresdefault.jpg)](https://youtube.com/shorts/U2rkALwhL6I?feature=share)
[![Session 2](https://img.youtube.com/vi/GvZ0LtC1vl4/maxresdefault.jpg)](https://youtube.com/shorts/GvZ0LtC1vl4?feature=share)
[![Session 3](https://img.youtube.com/vi/oZ7OU-YTM7g/maxresdefault.jpg)](https://youtube.com/shorts/oZ7OU-YTM7g?feature=share)

Each video shows YOLO-based vehicle detection running on images captured with the Basler camera under natural daylight conditions.

---

## Output Structure

Each CAN-triggered session generates a structured dataset containing:

* Raw captured images
* YOLO detection outputs
* Processed Excel files containing statistical measurements

Example repository structure:

```
AI_Robustness/
│
├── DATA/
│   ├── session_2026-01-19_09-22-39/
│   │   ├── images/
│   │   ├── detections/
│   │   ├── results.xlsx
│   │   └── session_metadata.json
│
├── scripts/
├── models/
└── README.md
```

### Generated Data

For each processed frame the system records:

| Parameter     | Description                       |
| ------------- | --------------------------------- |
| Timestamp     | Time of image capture             |
| Vehicle class | YOLO predicted class              |
| Confidence    | Detection probability             |
| Bounding box  | Object location coordinates       |
| Session ID    | Identifier of measurement session |

These datasets enable **post-processing and statistical evaluation** of detection robustness across multiple vehicles and repeated measurement sessions.

---

## Installation

### Requirements

* Python **3.9+**
* Basler camera with **pypylon SDK**
* CAN interface compatible with **python-can**
* Ultralytics YOLO framework

Install required dependencies:

```bash
pip install ultralytics pypylon python-can pandas openpyxl
```

---

## Running the System

1. Connect the Basler camera to the system.
2. Connect the CAN interface.
3. Start the detection pipeline:

```bash
python main.py
```

The system will initialize the camera and detection model, then wait for a **CAN trigger to start a measurement session**.

---

## CAN Session Control

**Start Session**

```
Arbitration ID: 0x7B
Signal: 2
```

**Stop Session**

```
Arbitration ID: 0x7B
Signal: 1
```

During an active session the system will:

* capture images from the Basler camera
* perform real-time YOLO detection
* store results in structured output files

---

## Research Context

This project was developed as part of a **Bachelor Thesis on the robustness of artificial intelligence in automotive perception systems**.

The goal of the research is to analyze how consistently modern object detection models detect vehicles under controlled measurement conditions.

Future work includes:

* comparative evaluation between **YOLOv10** and **YOLOv26**
* robustness testing under different environmental conditions
* analysis of detection stability across additional vehicle classes

---

## Author

**Hasib Kolaković**
Bachelor Thesis — Artificial Intelligence Robustness Evaluation

---

## License

This project is intended for **academic and research purposes**.
