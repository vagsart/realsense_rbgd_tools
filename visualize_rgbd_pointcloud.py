import open3d as o3d
import numpy as np
import cv2
import os
import sys

# Function to load the camera intrinsics from a text file
def load_intrinsics(filename):
    intrinsics = {}
    with open(filename, 'r') as f:
        lines = f.readlines()
        for line in lines:
            if line.startswith("Width"):
                intrinsics['width'] = int(line.split(":")[1].strip())
            elif line.startswith("Height"):
                intrinsics['height'] = int(line.split(":")[1].strip())
            elif line.startswith("Focal Length"):
                intrinsics['fx'], intrinsics['fy'] = map(float, line.split(":")[1].strip().split(","))
            elif line.startswith("Principal Point"):
                intrinsics['cx'], intrinsics['cy'] = map(float, line.split(":")[1].strip().split(","))
    return intrinsics

# Function to visualize the RGB and depth pair using Open3D
def visualize_rgbd_pair(depth_image_path, rgb_image_path, depth_intrinsics_path, rgb_intrinsics_path):
    # Load depth and RGB images
    depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)
    rgb_image = cv2.imread(rgb_image_path)

    # Convert RGB to RGB (Open3D expects RGB, not BGR)
    rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

    # Load the intrinsics for both depth and RGB
    depth_intrinsics = load_intrinsics(depth_intrinsics_path)
    rgb_intrinsics = load_intrinsics(rgb_intrinsics_path)

    # Resize depth image to match the RGB image size
    depth_image_resized = cv2.resize(depth_image, (rgb_intrinsics['width'], rgb_intrinsics['height']), interpolation=cv2.INTER_NEAREST)

    # Create Open3D images from numpy arrays
    depth_o3d = o3d.geometry.Image(depth_image_resized.astype(np.uint16))  # Depth image as 16-bit
    rgb_o3d = o3d.geometry.Image(rgb_image)

    # Create PinholeCameraIntrinsic objects from intrinsics
    intrinsic = o3d.camera.PinholeCameraIntrinsic()
    intrinsic.set_intrinsics(
        width=rgb_intrinsics['width'],
        height=rgb_intrinsics['height'],
        fx=rgb_intrinsics['fx'],
        fy=rgb_intrinsics['fy'],
        cx=rgb_intrinsics['cx'],
        cy=rgb_intrinsics['cy']
    )

    # Create RGBD image from RGB and depth images
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgb_o3d, depth_o3d, depth_scale=1000.0, convert_rgb_to_intensity=False
    )

    # Generate point cloud from the RGBD image
    point_cloud = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([point_cloud])


# Main execution
if __name__ == "__main__":
    # Specify the paths to the files
    if len(sys.argv) != 3:
        print("Usage: python visualize_rgbd_pointcloud.py <folder_path> <frame_num>")
        sys.exit(1)

    folder_path = sys.argv[1]
    frame_num = sys.argv[2]
    depth_image_path = os.path.join(folder_path, f'depth/d_{str(frame_num).zfill(4)}.png')
    rgb_image_path = os.path.join(folder_path, f'rgb/r_{str(frame_num).zfill(4)}.png')

    depth_intrinsics_path = os.path.join(folder_path, 'depth_intrinsics.txt')
    rgb_intrinsics_path = os.path.join(folder_path, 'rgb_intrinsics.txt')


    # Call the visualization function
    visualize_rgbd_pair(depth_image_path, rgb_image_path, depth_intrinsics_path, rgb_intrinsics_path)
