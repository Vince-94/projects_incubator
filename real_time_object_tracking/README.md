# Real-Time Object Detection and Tracking System

- [Real-Time Object Detection and Tracking System](#real-time-object-detection-and-tracking-system)
  - [Overview](#overview)
  - [Features](#features)
  - [Dependencies](#dependencies)
  - [Installation](#installation)
  - [Usage](#usage)
  - [Project Structure](#project-structure)
  - [Key Features in Detail](#key-features-in-detail)


## Overview

This project implements a real-time object detection and tracking system using Python, PyTorch, and OpenCV. It combines traditional computer vision techniques with deep learning to detect and track objects in high-resolution images or video streams. The system is optimized for performance and can be extended for various applications like surveillance, robotics, and autonomous vehicles.


## Features

- Object Detection: Uses a pre-trained YOLOv8 model (PyTorch) for accurate and fast object detection.
- Object Tracking: Implements the SORT (Simple Online and Realtime Tracking) algorithm to track objects across frames.
- Real-Time Processing: Optimized for real-time performance using GPU acceleration (CUDA) and multi-threading.
- Noise Reduction: Includes preprocessing techniques like Gaussian blur and median filtering to handle noisy input.
- Customizable: Easily extendable to support custom datasets and additional object classes.


## Dependencies
- OpenCV
- NumPy/SciPy
- PyTorch
- SORT
- CUDA


## Installation

```sh
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Download pre-trained YOLOv8 weights:
- Download the weights file (yolov8s.pt) from the official YOLOv8 repository (https://github.com/ultralytics/ultralytics).
- Place the file in the models/ directory.


## Usage

```sh
python main.py --input videos/sample.mp4 --output output.mp4
```


## Project Structure

```sh
real_time_object_tracking/
├── models/                  # Pre-trained YOLOv8 model weights
├── utils/                  # Utility functions (e.g., preprocessing, tracking)
├── config.py               # Configuration file for parameters
├── main.py                 # Main script to run the system
├── requirements.txt        # List of dependencies
├── README.md               # Project documentation
└── videos/                 # Sample input and output videos
```

## Key Features in Detail
1. Object Detection
- Uses YOLOv8 (PyTorch implementation) for fast and accurate object detection.
- Supports 80 COCO classes by default (e.g., person, car, dog).
- Easily extendable to custom datasets.

2. Object Tracking
- Implements the SORT algorithm to track objects across frames.
- Assigns unique IDs to detected objects and maintains their trajectories.

3. Real-Time Optimization
- Utilizes GPU acceleration (CUDA) for faster inference.
- Implements multi-threading for efficient video frame processing.

4. Noise Reduction
- Applies Gaussian blur and median filtering to reduce noise in input frames.
- Ensures robust detection and tracking in low-quality or noisy environments.


Future Work
- Add support for 3D object detection using LiDAR or depth sensors.
- Implement multi-camera tracking for large-scale surveillance.
- Extend the system to perform instance segmentation.
- Integrate with a robotics platform for real-world applications.

