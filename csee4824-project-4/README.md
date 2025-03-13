
# EECS 470 Final Project

Welcome to the EECS 470 Final Project!

This is the repository for your implementation of an out-of-order,
synthesizable, RISC-V processor with advanced features.

See the [Project Specification](https://drive.google.com/file/d/1z8MC70pnj0iMrgUmu1rYOlS5uGNltmwG/view?usp=drive_link)
for more details on deadlines and the overall structure of the project.

This README has information on changes from project 3 and specific
requirements on your processor for submitting to the autograder.

### Autograder Submission

For milestone 1, just submit the code and add commands below to both
run your testbench and view coverage for the testbench. We will be
grading it manually.

```
TODO: Add commands to run your single module testbench here

# To run and check the output:
TODO

# To view coverage:
TODO
```

For other autograder submissions, we require these three things:

- Running `make simv` will compile a simulation executable for your
  processor

- Running `make syn_simv` will compile a synthesis executable for your
  processor

- Running `./simv +MEMORY=program.mem` (or `./syn_simv`) will run the
  processory with the given program and output the `@@@` memory values
  as in project 3.

One note on memory ouput: when you start implementing your data cache,
you will need to ensure that any dirty cache values get written in the
memory output instead of the value from memory. This will require
exposing your cache at the top level and editing the `show_mem` task in
`test/pipeline_test.sv`.

## Getting Started

Start the project by working on your first module, either the ReOrder
Buffer (ROB) or the Reservation Station (RS). Implement the modules in
files in the `verilog/` folder, and write testbenches for them in the
`test/` folder. If you're writing the ROB, name these like:
`verilog/rob.sv` and `test/rob_test.sv` which implement and test the
module named `rob`.

Once you have something written, try running the new Makefile targets.
Add `rob` to the TESTED_MODULES variable in the Maekefile, then run
`make rob.pass` to compile, run, and check the testbench. Do the same
for synthesis with `make rob.syn.pass`. And finally, check your
testbench's coverage with `make rob.coverage.`

After you have the first module written and tested, keep going and work
towards a full processor. Plan to pass the `mult_no_lsq` program for the
second milestone (verify with the .wb file).

## Changes from Project 3

Many of the files from project 3 are still present or kept the same,
but there are a number of notable changes:

### The Makefile

The final project requires writing many modules, so we've added a new
section to the Makefile to compile arbitrary modules and testbenches.

To make it work for a module `mod`, create the files `verilog/mod.sv`
and `test/mod_test.sv` which implement and test the module. If you
update the `TESTED_MODULES` variable in the Makefile, then it will
be able to link the new targets below.

The most straightforward targets are `make mod.pass`,
`make mod.syn.pass` and `make mod.coverage`, which check if the module
passes the testbench in simulation, if it passes the testbench in
synthesis, and print the output of coverage for the module.

``` make
# ---- Module Testbenches ---- #
# NOTE: these require files like: 'verilog/rob.sv' and 'test/rob_test.sv'
#       which implement and test the module: 'rob'
make <module>.pass   <- greps for "@@@ Passed" or "@@@ Incorrect" in the output
make <module>.out    <- run the testbench (via <module>.simv)
make <module>.simv   <- compile the testbench executable
make <module>.verdi  <- run in verdi (via <module>.simv)
make <module>.syn.pass   <- greps for "@@@ Passed" or "@@@ Incorrect" in the output
make <module>.syn.out    <- run the synthesized module on the testbench
make <module>.syn.simv   <- compile the synthesized module with the testbench
make synth/<module>.vg   <- synthesize the module
make <module>.syn.verdi  <- run in verdi (via <module>.syn.simv)

# ---- module testbench coverage ---- #
make <module>.coverage    <- print the coverage hierarchy report to the terminal
make <module>.cov.verdi   <- open the coverage report in verdi
make <module>.cov         <- compiles a coverage executable for the module and testbench
make <module>.cov.vdb     <- runs the executable and creates the <module>.cov.vdb directory
make <module>_cov_report  <- run urg to create human readable coverage reports
```

### `verilog/sys_defs.svh`

`sys_defs` has received a few changes to prepare the final project:

1.  We've defined `CACHE_MODE`, affecting `test/mem.sv` and changing
    the way the processor interacts with memory.

2.  We've added a memory latency of 100ns, so memory is now much
    slower, and handling it with caching is necessary.

3.  There is a new 'Parameters' section giving you a starting point
    for some common macros that will likely need to be decided on like
    the size of the ROB, the number of functional units, etc.

### Pipeline Files

The two files `verilog/pipeline.sv` and `test/pipeline_test.sv` have
been edited to comment-out or remove project 3 specific code, so you
should be able to re-use them when you want to start integrating your
modules into a full processor again.

## New Files

We've added an `icache` module in `verilog/icache.sv`. That file has
more comments explaining how it works, but the idea is it stores
memory's response tag until memory returns that tag with the data. More
about how our processor's memory works will be presented in the final
lab section.

The file `psel_gen.sv` implements an incredibly efficient parameterized
priority selector (remember project 1?!). many tasks in superscalar
processors come down to priority selection, so instead of writing
manual for-loops, try to use this module. It is faster than any
priority selector the instructors are aware of (as far as my last
conversation about it with Brehob).

As promised, we've also copied the multiplier from project 2 and moved
the `` `STAGES`` definition to `sys_defs.svh` as `` `MULT_STAGES``.
This is set to 4 to start, but you can change it to 2 or 8 depending on
your processor's clock period.

### `verilog/p3` and the `decoder.sv`

The project 3 files are no longer relevant to your final processor, but
they are still good references, so project 3's starter verilog source
files have been moved to `verilog/p3/`. Notably, the decoder has been
pulled out as a new file `verilog/decoder.sv`.

## P3 Makefile Target Reference

This is the Makefile target reference from project 3, I've left it here
for reference. Most of the P3 portion of the Makefile is unchanged.

To run a program on the processor, run `make <my_program>.out`. This
will assemble a RISC-V `*.mem` file which will be loaded into `mem.sv`
by the testbench, and will also compile the processor and run the
program.

All of the "`<my_program>.abc`" targets are linked to do both the
executable compilation step and the `.mem` compilation steps if
necessary, so you can run each without needing to run anything else
first.

`make <my_program>.out` should be your main command for running
programs: it creates the `<my_program>.out`, `<my_program>.wb`, and
`<my_program>.ppln` output, writeback, and pipeline output files in the
`output/` directory. The output file includes the status of memory and
the CPI, the writeback file is the list of writes to registers done by
the program, and the pipeline file is the state of each of the pipeline
stages as the program is run.

The following Makefile rules are available to run programs on the
processor:

``` make
# ---- Program Execution ---- #
# These are your main commands for running programs and generating output
make <my_program>.out      <- run a program on simv
                              generate *.out, *.wb, and *.ppln files in 'output/'
make <my_program>.syn.out  <- run a program on syn_simv and do the same

# ---- Executable Compilation ---- #
make simv      <- compiles simv from the TESTBENCH and SOURCES
make syn_simv  <- compiles syn_simv from TESTBENCH and SYNTH_FILES
make *.vg      <- synthesize modules in SOURCES for use in syn_simv
make slack     <- grep the slack status of any synthesized modules

# ---- Program Memory Compilation ---- #
# Programs to run are in the programs/ directory
make programs/<my_program>.mem  <- compile a program to a RISC-V memory file
make compile_all                <- compile every program at once (in parallel with -j)

# ---- Dump Files ---- #
make <my_program>.dump  <- disassembles compiled memory into RISC-V assembly dump files
make *.debug.dump       <- for a .c program, creates dump files with a debug flag
make dump_all           <- create all dump files at once (in parallel with -j)

# ---- Verdi ---- #
make <my_program>.verdi     <- run a program in verdi via simv
make <my_program>.syn.verdi <- run a program in verdi via syn_simv

# ---- Visual Debugger ---- #
make <my_program>.vis  <- run a program on the project 3 vtuber visual debugger!
make vis_simv          <- compile the vtuber executable from VTUBER and SOURCES

# ---- Cleanup ---- #
make clean            <- remove per-run files and compiled executable files
make nuke             <- remove all files created from make rules
```
