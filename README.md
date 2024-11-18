# Bag File Processing and Visualization with RealSense and Open3D

This repository contains two scripts for working with Intel RealSense cameras. The first script extracts depth and RGB frames from a `.bag` file while saving camera intrinsic and calibration parameters. The second script visualizes depth and RGB data as a 3D point cloud using Open3D.

---

## 1. **`realsense_bag_to_frames.py`**
### Description:
Processes an Intel RealSense `.bag` file to:
- Extract depth and RGB frames.
- Save camera intrinsics for depth and RGB streams.
- Save additional camera parameters, such as depth scale and baseline.

### Usage:
Run the script with the following command:
```bash
python realsense_bag_to_frames.py <bagfile> <output_folder>
```

### Example:
```bash
python realsense_bag_to_frames.py my_recording.bag output_folder
```

### Outputs:
- Extracted depth frames in `<output_folder>/depth/`.
- Extracted RGB frames in `<output_folder>/rgb/`.
- Camera intrinsics:
  - `depth_intrinsics.txt`: Intrinsics for the depth stream.
  - `rgb_intrinsics.txt`: Intrinsics for the RGB stream.
- Additional parameters:
  - `additional_params.txt`: Contains depth scale, baseline, and `baseline * fx`.

---

## 2. **`visualize_rgbd_pointcloud.py`**
### Description:
Visualizes a pair of RGB and depth images using Open3D by:
- Loading camera intrinsics from previously saved `.txt` files.
- Creating an RGBD image.
- Generating and displaying a 3D point cloud.

### Prerequisites:
Ensure the output files from `realsense_bag_to_frames.py` (depth, RGB images, and intrinsics) are available.

### Usage:
pass the file paths in the script o to visualize the data.

### Run the Script:
```bash
python visualize_rgbd_pointcloud.py /path/to/folder frame_num
```

---

## Dependencies:
Both scripts rely on the following Python libraries:
- **Intel RealSense SDK (`pyrealsense2`)**
- **OpenCV (`cv2`)**
- **Open3D**

### Install Requirements:
```bash
pip install pyrealsense2 opencv-python-headless open3d
```

---

## Workflow:
1. **Extract Frames**: Use `realsense_bag_to_frames.py` to process the `.bag` file and extract depth/RGB frames and camera parameters.
2. **Visualize**: Use `visualize_rgbd_pointcloud.py` to visualize and create a point cloud from specific depth and RGB frames.

---

