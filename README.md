CAN-Triggered Real-Time Vehicle Detection for Robustness Evaluation
Abstract

This project presents a real-time vehicle detection framework developed for the robustness evaluation of YOLO-based object detection models in controlled automotive scenarios. The system integrates industrial image acquisition with CAN-based session triggering to enable structured and repeatable measurement procedures.

The current implementation is based on YOLOv10n, with planned comparative evaluation using YOLOv26.

Experimental Setup
Image Acquisition

Images are acquired using a Basler industrial camera via the pypylon SDK.
The camera operates at 5 FPS and records raw images exclusively during CAN-triggered measurement sessions.

Session Control

Measurement sessions are controlled via CAN bus communication:

Arbitration ID: 0x7B

Signal 2 → Start session

Signal 1 → Stop session

This design ensures deterministic and reproducible recording conditions.

Object Detection

Vehicle detection is performed using the Ultralytics implementation of YOLOv10n with a confidence threshold of 0.5.
For each detection, the following parameters are stored:

Class label

Confidence score

Bounding box coordinates

Detection results are exported to structured Excel files for statistical analysis.

Test Vehicles

Robustness evaluation was conducted using six vehicles with diverse geometrical characteristics:

Mercedes EQE

Škoda Enyaq

Mazda MX-5

Seat Cordoba

Opel Movano

Mercedes-Benz G-Klasse (Puch variant)

These vehicles were repeatedly measured to analyze:

Confidence stability

Detection consistency

Class robustness across varying vehicle shapes and sizes

Demonstration Videos

The repository includes three experimental recordings:

- [output2.mp4](./output2.mp4)
- [output3.mp4](./output3.mp4)
- [output6.mp4](./output6.mp4)


Each video demonstrates real-time detection performance under natural daylight conditions using the Basler camera setup.

Research Objective

The objective of this bachelor thesis is:

To conduct a comparative robustness analysis of YOLO-based vehicle detection models under controlled automotive measurement conditions.

Future work includes integration of YOLOv26 and statistical cross-model comparison based on confidence distributions and detection reliability metrics.
