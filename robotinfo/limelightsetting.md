# Pipelines

## Input

| Setting Name         | Value            |
|----------------------|------------------|
| Pipeline type        | AprilTags        |
| Source Image         | Camera           |
| Resolution           | 1280x960 40fps   |
| LEDs                 | On               |
| LED Power            | 100              |
| Stream Orientation   | Normal           |
| Exposure (.01 ms)    | 120              |

### Advanced

| Setting Name         | Value            |
|----------------------|------------------|
| Black Level Offset   | 0                |
| Sensor Gain          | 6                |
| Flicker Correction   | None             |
| Red Balance          | 1200             |
| Blue Balance         | 1975             |

---

## Configuration

| Setting Name         | Value                              |
|----------------------|------------------------------------|
| Family               | AprilTag Classic 36h11 (587 tags)  |
| Engine               | U-Michigan                         |
| Marker Size (mm)     | 165.1                              |
| Detector Downscale   | 2                                  |
| Quality Threshold    | 2                                  |
| ID Filters           | (blank)                            |
| X-Crop               | -1 to 1                            |
| Y-Crop               | -1 to 1                            |
| H-Keystone           | 0                                  |
| Y-Keystone           | 0                                  |

### Multi-Target Sorting and Grouping

| Setting Name         | Value                |
|----------------------|----------------------|
| Sort Mode            | Largest              |
| Area (% of image)    | 0.0010 to 100.0000   |
| Target Grouping      | Single Target        |

---

## Advanced

| Setting Name              | Value   |
|---------------------------|---------|
| Full 3D Targeting         | ✔       |
| Gyro Latency Adjustment   | 0       |

### 3D Point-Of-Interest Offset

| Setting Name   | Value   |
|----------------|---------|
| Forward (m)    | 0       |
| Right (m)      | 0       |
| Up (m)         | 0       |

### MegaTag Field-Space Localization Setup

| Setting Name         | Value          |
|----------------------|----------------|
| LL Forward           | 0              |
| LL Roll              | 0              |
| LL Right             | 0              |
| LL Pitch             | 0              |
| LL Up                | 0              |
| LL Yaw               | 0              |
| Snap Robot to Floor  | ✘              |
| Field Map File       | ✔              |

### Visualizer Setup

| Setting Name     | Value                        |
|------------------|------------------------------|
| View             | Robot Pose In Field Space    |
| Bot Width        | 0.7112                       |
| Bot Length        | 0.7112                      |
| Hide Info        | ✘                            |
| Target Info      | ✔                            |
| MT1 (Yellow)     | ✔                            |
| MT2 (Green)      | ✔                            |

---

## Output & Crosshair

| Setting Name       | Value              |
|--------------------|--------------------|
| Send Raw Corners   | ✘                  |
| Send JSON over NT  | ✘                  |
| Crosshair Mode     | Single Crosshair   |

### Crosshair A

| Setting Name   | Value   |
|----------------|---------|
| Reset XY       |         |
| X              | 0       |
| Y              | 0       |

### Hardware Panning

| Setting Name   | Value   |
|----------------|---------|
| Pan X          | 0       |
| Pan Y          | 0       |

---

# Settings

| Setting Name                                         | Value                              |
|------------------------------------------------------|------------------------------------|
| First Robotics Team Number / Net ID                  | 9036                               |
| USB Index (FIRST Tech Challenge, ROS)                | 0                                  |
| Control Hub/Linux/Mac/Chromebook USB Ethernet IP     | 172.29.0.1                         |
| Windows USB Ethernet IP Address                      | 172.28.0.1                         |
| Stream Rate                                          | Normal                             |
| Stream Resolution                                    | 320x240                            |
| IP Address Assignment                                | Static                             |
| IP Address                                           | 10.90.36.15                        |
| Netmask                                              | 255.255.255.0                      |
| Gateway                                              | 10.90.36.4                         |
| Hostname (lowercase letters only)                    | limelight-fixed                    |
| Preview                                              | http://limelight-fixed.local:5801  |
| Custom NT Server IP                                  | 0.0.0.0                            |
| Modbus Port                                          | 502                                |

---

# Calibration

> **USING CUSTOM CALIBRATION**

## ChArUco Calibration

| Setting Name             | Value                |
|--------------------------|----------------------|
| Grid Width               | 11                   |
| Grid Height              | 8                    |
| Square Size (mm)         | 20                   |
| Charuco Marker Size (mm) | 15                   |
| ArUco Dictionary         | 5x5, 250 markers     |
| ID Offset (Advanced)     | 0                    |