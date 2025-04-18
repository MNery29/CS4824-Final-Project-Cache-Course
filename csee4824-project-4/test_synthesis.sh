#!/bin/bash

# all modules to test:
# "pipeline.sv"

# missing test benches:
# "booth_mult_stage" "decoder" "h3hash" "icache" "lsq" "mult_stage" "psel_gen" "regfile" 
# "stage_cp" "stage_ex" "stage_mem" "stage_rt" "zcache"

# ========= Results Post-test! =====================================================

# UNITS PASSING ALL TESTS! <3 YAYY: "reorder_buffer" "cdb" "dcache" "map_table" "reservation_station"
# STAGES PASSING ALL TEST!!! YAYYY: "stage_id" "stage_if" "stage_is"

# ONES WHERE I NEED SOME CLARIFICATION: 
# --> mult, whats going on? what modules are actually used

TESTED_MODULES=("reorder_buffer" "cdb" "dcache" "map_table" "reservation_station" "stage_id" "stage_if" "stage_is")

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
