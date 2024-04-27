#!/bin/bash

data_dir="$1"

# Check if obj_dir is provided
if [ -z "$data_dir" ]; then
    echo "Usage: $0 <obj_dir>"
    exit 1
fi

obj_files=$(find "$data_dir" -type f -name "*.obj")
scn_files=$(find "$data_dir" -type f -name "*.scn")

for obj_file in $obj_files; do
    obj_file="${obj_file#./data/}"
    bin_file="${obj_file%.obj}.bin"

    # Check if the bin file doesn't exist or if it is older than the obj file
    if [ ! -e "$data_dir/$bin_file" ] || [ "$data_dir/$bin_file" -ot "$data_dir/$obj_file" ]; then
        echo "Building $bin_file from $obj_file"
        ./bin/obj "$obj_file" "$bin_file"
    else
        echo "Skipping $bin_file (up to date)"
    fi
done

for scn_file in $scn_files; do
    scn_file="${scn_file#$scn_dir/}"
    xyz_file="${scn_file%.scn}.xyz"
    oct_file="${scn_file%.scn}.oct"

    # Check if the corresponding .xyz file doesn't exist or is older than the .scn file
    if [ ! -e "$data_dir/$xyz_file" ] || [ "$data_dir/$xyz_file" -ot "$data_dir/$scn_file" ]; then
        echo "Running point_cloud.py for $scn_file"
        python3 point_cloud.py "$data_dir/$scn_file" "$data_dir" 1000
    else
        echo "Skipping $xyz_file (up to date)"
    fi
    if [ ! -e "$data_dir/$oct_file" ] || [ "$data_dir/$oct_file" -ot "$data_dir/$scn_file" ]; then
        echo "Running octree.py for $scn_file"
        python3 octree.py "$data_dir/$scn_file"
    else
        echo "Skipping $oct_file (up to date)"
    fi
done
