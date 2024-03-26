#!/bin/bash
folder_path="/home/cuong/CLionProjects/uMV-FSTSP/Murray_Chu_2015_test_data/FSTSP/FSTSP_10_customer_problems"
binary_file="/home/cuong/CLionProjects/uMV-FSTSP/cmake-build-release/uMV-FSTSP"

# Use find to get a list of all directories in the specified folder
# The -maxdepth 1 option ensures that only immediate subdirectories are considered
find "$folder_path" -maxdepth 1 -type d | while IFS= read -r folder; do
    # Extract the folder name from the full path
    folder_name=$(basename "$folder")
    output="/home/cuong/CLionProjects/uMV-FSTSP/og_murray_logs/${folder_name}.out"
    touch output
    # Call the binary file with the folder name as an argument
    "$binary_file" "$folder_name" 2>&1 | tee "$output"
done