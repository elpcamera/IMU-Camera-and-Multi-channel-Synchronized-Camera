# Linux x86_64 Standalone Executable Applications User Guide

This directory contains standalone portable executable applications packaged in **AppImage** format, designed for 64-bit x86 Linux distributions (Ubuntu 20.04 and above, such as Ubuntu 20.04 / 22.04 / 24.04).

---

## Software Overview

| Executable File | Description & Use Cases |
| :--- | :--- |
| **`x86_64-Global Camera Image IMU-v2.2.1.AppImage`** | **Monocular Camera Image + IMU Testing Tool**<br>Supports real-time image preview and exposure time adjustment for a single global shutter camera, as well as synchronized acquisition of 11 sets of IMU sensor data (accelerometer, gyroscope, hardware microsecond-level timestamps), terminal logging, and snapshot capture via hotkey/button. |
| **`x86_64-Multi channel synchronization testing software (binocular+monocular)-v2.2.1.AppImage`** | **Multi-Channel Synchronization Testing Tool (Binocular + Monocular Edition)**<br>Specifically designed for combined setups featuring a "binocular stereo camera module + independent monocular camera". Provides high-precision hardware microsecond-level timestamp synchronization, channel management, and multi-stream sync performance evaluation. |
| **`x86_64-Multi channel synchronization testing software-v2.2.1.AppImage`** | **Multi-Channel Synchronization Testing Tool (Standard Multi-Camera Edition)**<br>Supports hardware microsecond-level timestamp synchronization for multiple independent cameras (2 to 8 channels), real-time sync status monitoring, per-channel debugging, parameter configuration, and frame sync drop-rate detection. |

---

## Quick Start Guide

### Step 1: Grant Execution Permission

AppImage files do not have execution permissions enabled by default after downloading or copying. You must grant executable permission before running:

#### Method A: Command Line Terminal (Recommended)
Open a terminal in the current directory and run:
```bash
chmod +x *.AppImage
```

#### Method B: Desktop GUI
1. Right-click the `.AppImage` file and select **Properties**.
2. Switch to the **Permissions** tab.
3. Check the box for **"Allow executing file as program"**.

---

### Step 2: Launch the Application

#### Method A: Run via Terminal
```bash
# 1. Run Monocular Camera Image + IMU Testing Tool
./"x86_64-Global Camera Image IMU-v2.2.1.AppImage"

# 2. Run Binocular + Monocular Multi-Channel Synchronization Testing Tool
./"x86_64-Multi channel synchronization testing software (binocular+monocular)-v2.2.1.AppImage"

# 3. Run Standard Multi-Channel Synchronization Testing Tool
./"x86_64-Multi channel synchronization testing software-v2.2.1.AppImage"
```

#### Method B: Run via Graphical Desktop
In your desktop file manager, simply **double-click** the corresponding `.AppImage` file to launch the graphical user interface.

---

## Troubleshooting

### 1. Missing FUSE Error on Ubuntu 22.04 / 24.04
- **Error Message**:
  ```text
  dlopen(): error loading libfuse.so.2
  AppImages require FUSE to run.
  ```
- **Cause**: Newer Ubuntu releases (22.04 and later) no longer come with `libfuse2` pre-installed.
- **Solution**: Install the `libfuse2` package via terminal:
  ```bash
  sudo apt update
  sudo apt install -y libfuse2
  ```
  *(Note: For Ubuntu 24.04 and later, run `sudo apt install -y libfuse2t64`)*

- **Alternative (Run without installing dependencies)**: Append the `--appimage-extract-and-run` parameter at launch:
  ```bash
  ./"x86_64-Global Camera Image IMU-v2.2.1.AppImage" --appimage-extract-and-run
  ```

---

### 2. Camera Device Access Permission (Permission Denied)
- **Symptom**: The application cannot detect camera devices or fails to open video streams upon launch.
- **Cause**: The current user lacks read/write permissions for Linux video capture nodes (`/dev/video*`) or serial/USB device ports.
- **Solution**:
  Add the current user to the `video` and `dialout` groups:
  ```bash
  sudo usermod -aG video,dialout $USER
  ```
  > **Note**: After executing this command, you must **log out and log back in** (or reboot your computer) for group membership changes to take effect. For quick temporary testing, you can also launch the application with `sudo`:
  ```bash
  sudo ./"x86_64-Global Camera Image IMU-v2.2.1.AppImage"
  ```

---

## System Requirements
- **Operating System**: Linux x86_64 (Ubuntu 20.04 and above, such as Ubuntu 20.04 / 22.04 / 24.04 Desktop recommended).
- **USB Interface**: Motherboard native **USB 3.0 (blue port)** or higher is strongly recommended. Sufficient USB host controller bandwidth is required for multi-channel synchronous streaming.
