#!/bin/bash

# all modules to test:
# "pipeline.sv"

# missing test benches:
# "booth_mult_stage" "decoder" "h3hash" "lsq" "mult_stage" "psel_gen" "regfile" 
# "zcache" 
# "stage_ex" "stage_mem" "stage_rt"

# ========= Results Post-test! =====================================================

# UNITS PASSING ALL TESTS! <3 YAYY: "reorder_buffer" "cdb" "dcache" "map_table" "reservation_station" "icache"
# STAGES PASSING ALL TEST!!! YAYYY: "stage_id" "stage_if" "stage_is" "stage_cp"
#("reorder_buffer" "cdb" "dcache" "map_table" "reservation_station" "icache" "stage_id" "stage_if" "stage_is")

# ONES WHERE I NEED SOME CLARIFICATION: 
# --> mult, whats going on? what modules are actually used
# --> zcache, genvar issues i think im really confused lowkey 

TESTED_MODULES=("stage_ex")

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
