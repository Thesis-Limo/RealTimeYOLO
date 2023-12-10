# Real time YOLOv8 object detection for the LIMO

This README outlines the steps to run the ROS-based camera feed caster and the Python-based YOLOv8 image receiver on the LIMO robot.

## Prerequisites

- ROS Melodic installed and configured (specifically with the `astra_camera` package for the Dabai U3 camera).
- Python 2.7 environment for ROS scripts.
- Python 3.8 environment for YOLOv8.
- The camera feed caster (`camera_feed_caster.py`) and receiver scripts (`camera_feed_receiver.py`) are present in your working directory.

## Setup

### Creating Python 3.8 Virtual Environment

In your project directory:

```bash
python3.8 -m venv .venv
source .venv/bin/activate
# Update pip
pip install --upgrade pip
# Install necessary packages
pip install -r requirements.txt
```

_Note: Ensure you have Python 3.8 installed on your system._

## Running the Scripts

### Step 1: Launch ROS Camera Node

1. **Open a new terminal**.
2. Launch the camera:
   ```bash
   roslaunch astra_camera dabai_u3.launch
   ```

### Step 2: Start the Camera Feed Caster

1. **Open a second terminal**.
2. Run the camera feed caster script with Python 2.7:
   ```bash
   python2.7 camera_feed_caster.py
   ```

### Step 3: Start the Image Receiver

1. **Open a third terminal**.
2. Activate the Python 3.8 virtual environment:
   ```bash
   source .venv/bin/activate
   ```
3. Limit OpenMP threads (needed for performance)
   ```
   export OMP_NUM_THREADS=1
   ```
4. Run the camera feed receiver script with Python 3.8:
   ```bash
   python3.8 camera_feed_receiver.py
   ```

## Notes

- Ensure each component is fully operational before moving to the next step.
- Keep the camera feed caster script running to maintain the camera feed to the receiver.
- To stop the scripts, use `Ctrl+C` in each terminal.
