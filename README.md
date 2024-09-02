# Object Detection-Based Pick and Place Sorting with Manipulator

## Prerequisites

To run the system on a Jetson device, follow the steps below to set up the required environment:
- Install Ubuntu 20.04 on Jetson:
Flash your Jetson device with the Ubuntu 20.04 OS image by following this guide: [Jetson Nano Ubuntu 20.04 Image](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image?tab=readme-ov-file).
This image comes pre-installed with Python 3.8, OpenCV 4.8 with CUDA enabled, and all necessary CUDA drivers and libraries for optimized performance on Jetson.

- Install OpenCV with CUDA Enabled:
The above OS image ensures that OpenCV 4.8 is installed with CUDA support, enabling accelerated image processing and deep learning tasks, crucial for running object detection efficiently on the Jetson hardware.

- Set Up a Python Environment:
Create a dedicated Python environment to manage your project dependencies, ensuring a clean and organized setup.


Before installing the required packages, make sure you have YOLOv5 installed. Follow the steps below to set it up:
- Clone the YOLOv5 repository:
git clone https://github.com/ultralytics/yolov5.git
cd yolov5

- Install YOLOv5 dependencies:
pip install -r requirements.txt

- Custom Train the Model:
Once you have YOLOv5 installed, you can follow the instructions on their GitHub repository to custom train your model with your own dataset and obtain the weights. After training, you will have a set of custom weights that can be used with your project.


## Installation

- After setting up YOLOv5, you can install the rest of the project dependencies by running the following command from the root of your project directory:
pip install -r requirements.txt

## Usage

To use the system, follow these steps:
- Enable Torque of the Arm Motors:
Start by running controller.py, which enables the torque of the robotic arm motors. You can modify the initial joint angles directly within the controller.py file as needed:

- Adjust Transformations:
Before running the pick-and-place code, make any necessary adjustments to the transformations in the code to ensure proper alignment and functioning of the robotic arm.

- Run Object Detection and Pick-and-Place:

After making the adjustments, run the pnp_with_OD.py script. This script detects the custom-trained object using the YOLOv5 model and performs the pick-and-place operation:
python pnp_with_OD.py
