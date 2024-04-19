obj_dir="./data"

obj_files=$(find "$obj_dir" -type f -name "*.obj")

for obj_file in $obj_files; do
    obj_file="${obj_file#./data/}"
    bin_file="${obj_file%.obj}.bin"
    echo $bin_file $obj_file
    ./build/obj "$obj_file" "$bin_file"
done