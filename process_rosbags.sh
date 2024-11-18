#!/bin/bash

# Loop through each .bag file in ./data/rosbags
for bagfile in ./data/rosbags/*.bag
do
    # Get the filename without the .bag extension
    filename=$(basename "$bagfile" .bag)
    
    # Create the output directory under ./data/processed/
    output_folder="./data/processed/$filename"
    
    # Create the directory if it doesn't exist
    mkdir -p "$output_folder"
    
    # Run the Python script for the current .bag file and specify the output folder
    python realsense_bag_to_frames.py "$bagfile" "$output_folder"
    
    # Optional: Print message when done processing each file
    echo "Processed $bagfile and saved to $output_folder"
done

