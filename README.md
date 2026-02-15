# 🚗 CAN-Triggered Real-Time Vehicle Detection for Robustness Evaluation

## Abstract

This project presents a real-time vehicle detection framework developed for robustness evaluation of YOLO-based object detection models in controlled automotive scenarios. The system integrates industrial image acquisition with CAN-based session triggering to enable structured and repeatable measurement procedures.

The current implementation is based on **YOLOv10n**, with planned comparative evaluation using **YOLOv26**.

---

## Experimental Setup

### Image Acquisition

Images are acquired using a Basler industrial camera via the `pypylon` SDK.  
The camera operates at 5 FPS and records raw images exclusively during CAN-triggered measurement sessions.

### Session Control (CAN Bus)

Measurement sessions are controlled via CAN communication:

- Arbitration ID: `0x7B`  
- Signal `2` → Start session  
- Signal `1` → Stop session  

This ensures deterministic and reproducible recording conditions similar to real automotive environments.

### Object Detection

Vehicle detection is performed using the Ultralytics implementation of **YOLOv10n** with a confidence threshold of 0.5.

For each detection, the following parameters are stored:

- Class label  
- Confidence score  
- Bounding box coordinates  

All results are exported to structured Excel files for statistical robustness analysis.

---

## Test Vehicles

Robustness evaluation was conducted using six vehicles with diverse geometrical characteristics:

- Mercedes EQE  
- Škoda Enyaq  
- Mazda MX-5  
- Seat Cordoba  
- Opel Movano  
- Mercedes-Benz G-Klasse (Puch variant)  

Repeated measurements were performed to analyze:

- Confidence stability  
- Detection consistency  
- Class robustness across varying vehicle shapes and sizes  

---

## Demonstration Videos

[![Session 1](https://img.youtube.com/vi/U2rkALwhL6I/maxresdefault.jpg)](https://youtube.com/shorts/U2rkALwhL6I?feature=share)
[![Session 2](https://img.youtube.com/vi/GvZ0LtC1vl4/maxresdefault.jpg)](https://youtube.com/shorts/GvZ0LtC1vl4?feature=share)
[![Session 3](https://img.youtube.com/vi/oZ7OU-YTM7g/maxresdefault.jpg)](https://youtube.com/shorts/oZ7OU-YTM7g?feature=share)



Each video shows YOLO-based vehicle detection running on images captured with the Basler camera under natural daylight conditions.

---

## Output Structure

Each CAN-triggered session generates:

