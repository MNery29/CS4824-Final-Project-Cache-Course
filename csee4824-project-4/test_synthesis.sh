#!/bin/bash

# List of modules to be tested
# failing simv - mult rob cdb
# failing synth - map_table
# failing synthesis test (port mismatch) - reorder_buffer, stage_id
TESTED_MODULES=("reservation_station" "dcache")

# Loop through each module and run the make commands
for module in "${TESTED_MODULES[@]}"
do
    echo "===== Running for module: $module ====="
    
    make "$module.out" || { echo "Failed: make $module.out"; exit 1; }
    make "synth/$module.vg" || { echo "Failed: make synth/$module.vg"; exit 1; }
    make "$module.syn.out" || { echo "Failed: make $module.syn.out"; exit 1; }
    make slack || { echo "Failed: make slack"; exit 1; }

    echo "===== Completed: $module ====="
    echo
done

echo "All modules processed."
