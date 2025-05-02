#!/bin/bash

# this sh script is used to run the synthesis tests for the modules in the project
# it will run the make commands for each module and check if they pass or fail
# it creates the synthesized module for each module and checks if the synthesized module passes the testbench
# it will also check if the slack violations are within the limits

# list of all modules to be tested:
# stages: stage_id, stage_if, stage_is, stage_cp, stage_ex, stage_mem, stage_rt
# internal modules: 

# ========= Results Post-test! ===================================================================

# UNITS WITH SYNTAX ERRORS: "stage_id" "stage_rt"

# STAGES PASSING (I think, some are not passing testbenches but I think it is because we haven't updated the testbenches yet): 
# "stage_if" "stage_is" "stage_cp" "stage_ex"

# STAGES WITH NO TESTBENCHES: "stage_mem"

TESTED_MODULES=("pipeline")

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
