# Blender2Dynamixel
> Export Blender animations and control Dynamixel motors in real-time

## 📋 Overview
A system that exports animations from Blender and reproduces them on physical Dynamixel servo motors.

## 🔧 Files

### `dynamixel_blender_keyframe_export.py`
- Blender script for exporting armature animations to JSON
- Supports multiple armatures and motor configurations
- Run inside Blender to export keyframe data

### `dynamixel_control.py`
- Real-time Dynamixel motor control system
- Reads exported JSON and controls multiple motors simultaneously
- Includes speed optimization and error monitoring

## 🛠️ Hardware Requirements
- Dynamixel servo motors
- USB2Dynamixel or U2D2 interface
- Appropriate power supply for motors
- **Set motors to Extended Position Mode**

## 📦 Software Requirements
- **Blender 3.0+** (for animation export)
- **Python 3.7+** with packages:
  - `dynamixel_sdk`
  - `numpy`
  - `matplotlib`

## 🎥 Usage Guide
Detailed setup and usage instructions are available in the video tutorial.