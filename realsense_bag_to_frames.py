import pyrealsense2 as rs
import numpy as np
import cv2
import os
import sys

# Function to create directory if it doesn't exist
def create_directory(path):
    if not os.path.exists(path):
        os.makedirs(path)

# Function to save the camera intrinsics to a text file
def save_intrinsics(intrinsics, filename):
    with open(filename, 'w') as f:
        f.write(f"Width: {intrinsics.width}\n")
        f.write(f"Height: {intrinsics.height}\n")
        f.write(f"Focal Length (fx, fy): {intrinsics.fx}, {intrinsics.fy}\n")
        f.write(f"Principal Point (cx, cy): {intrinsics.ppx}, {intrinsics.ppy}\n")
        f.write(f"Distortion Model: {intrinsics.model}\n")
        f.write(f"Distortion Coefficients: {intrinsics.coeffs}\n")

# Function to save additional depth and calibration parameters
def save_additional_params(pipeline, filename):
    profile = pipeline.get_active_profile()
    device = profile.get_device()

    depth_sensor = device.first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()  # Depth scale (meters per unit)

    # Get depth and color stream profiles
    depth_stream_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    color_stream_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()

    # Get the extrinsics between depth and color streams
    depth_to_color_extrinsics = depth_stream_profile.get_extrinsics_to(color_stream_profile)

    # The extrinsics object has a translation attribute for the translation vector
    translation = depth_to_color_extrinsics.translation
    baseline = np.linalg.norm(translation)  # Baseline in meters, magnitude of translation vector

    with open(filename, 'w') as f:
        f.write(f"Depth Scale (meters per unit): {depth_scale}\n")
        f.write(f"Baseline (meters): {baseline}\n")
        f.write(f"Baseline times fx (depth baseline * fx): {baseline * depth_scale}\n")

def custom_rsconvert(bagfile, folder_name):
    # Create output directories
    depth_folder = os.path.join(folder_name, 'depth')
    rgb_folder = os.path.join(folder_name, 'rgb')
    create_directory(depth_folder)
    create_directory(rgb_folder)

    # Initialize the pipeline and configure it to read the bag file
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device_from_file(bagfile)

    # Start the pipeline
    pipeline.start(config)

    # Get the first set of frames to access the intrinsics
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    # Get the intrinsics for both the depth and color (RGB) streams
    depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
    color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

    # Save intrinsics to text files
    save_intrinsics(depth_intrinsics, os.path.join(folder_name, 'depth_intrinsics.txt'))
    save_intrinsics(color_intrinsics, os.path.join(folder_name, 'rgb_intrinsics.txt'))

    # Save additional parameters to a text file
    save_additional_params(pipeline, os.path.join(folder_name, 'additional_params.txt'))

    # Initialize frame counters
    depth_counter = 0
    rgb_counter = 0

    # Process the frames and save images
    try:
        while True:
            # Wait for the next frame set
            frames = pipeline.wait_for_frames()
            
            # Get depth and color frames
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            # Convert the depth frame to a numpy array and save it as grayscale
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_image = depth_image.astype(np.uint16)  # Ensure depth is 16-bit
            depth_filename = os.path.join(depth_folder, f"depth_{depth_counter:04d}.png")
            cv2.imwrite(depth_filename, depth_image)

            # Convert the color frame to a numpy array and save it as RGB
            rgb_image = np.asanyarray(color_frame.get_data())
            rgb_filename = os.path.join(rgb_folder, f"rgb_{rgb_counter:04d}.png")
            cv2.imwrite(rgb_filename, rgb_image)

            # Increment the counters
            depth_counter += 1
            rgb_counter += 1

    except KeyboardInterrupt:
        print("Processing interrupted. Stopping the pipeline.")
    
    finally:
        # Stop the pipeline after extracting all frames
        pipeline.stop()

    print("Images and intrinsics have been saved successfully.")

# Main script execution
if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python realsense_bat_to_frames.py <bagfile> <output_folder>")
        sys.exit(1)

    bagfile = sys.argv[1]
    folder_name = sys.argv[2]

    custom_rsconvert(bagfile, folder_name)
