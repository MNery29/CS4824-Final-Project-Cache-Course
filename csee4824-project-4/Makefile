##########################
# ---- Introduction ---- #
##########################

# Welcome to the Project 3 VeriSimpleV Processor makefile!
# this file will build and run a fully synthesizable RISC-V verilog processor
# and is an extended version of the CSEE 4824 standard makefile

# NOTE: this file should need no changes for project 3
# but it will be reused for project 4, where you will likely add your own new files and functionality

# reference table of all make targets:

# make  <- runs the default target, set explicitly below as 'make no_hazard.out'
.DEFAULT_GOAL = no_hazard.out
# ^ this overrides using the first listed target as the default

# ---- Module Testbenches ---- #
# NOTE: these require files like: 'verilog/rob.sv' and 'test/rob_test.sv'
#       which implement and test the module: 'rob'

# make <module>.pass   <- greps for "@@@ Passed" or "@@@ Incorrect" in the output
# make <module>.out    <- run the testbench (via <module>.simv)
# make <module>.simv   <- compile the testbench executable
# make <module>.verdi  <- run in verdi (via <module>.simv)
# make <module>.syn.pass   <- greps for "@@@ Passed" or "@@@ Incorrect" in the output
# make <module>.syn.out    <- run the synthesized module on the testbench
# make <module>.syn.simv   <- compile the synthesized module with the testbench
# make synth/<module>.vg   <- synthesize the module
# make <module>.syn.verdi  <- run in verdi (via <module>.syn.simv)

# ---- module testbench coverage ---- #
# make <module>.coverage    <- print the coverage hierarchy report to the terminal
# make <module>.cov.verdi   <- open the coverage report in verdi
# make <module>.cov         <- compiles a coverage executable for the module and testbench
# make <module>.cov.vdb     <- runs the executable and creates the <module>.cov.vdb directory
# make <module>_cov_report  <- run urg to create human readable coverage reports

# ---- Program Execution ---- #
# these are your main commands for running programs and generating output
# make <my_program>.out      <- run a program on simv and generate .out, .wb, and .ppln files in 'output/'
# make <my_program>.syn.out  <- run a program on syn_simv and do the same
# make simulate_all          <- run every program on simv at once (in parallel with -j)
# make simulate_all_syn      <- run every program on syn_simv at once (in parallel with -j)

# ---- Executable Compilation ---- #
# make simv      <- compiles simv from the TESTBENCH and SOURCES
# make syn_simv  <- compiles syn_simv from TESTBENCH and SYNTH_FILES
# make *.vg      <- synthesize modules in SOURCES for use in syn_simv
# make slack     <- grep the slack status of any synthesized modules

# ---- Program Memory Compilation ---- #
# NOTE: programs to run are in the programs/ directory
# make programs/<my_program>.mem  <- compiles a program to a RISC-V memory file for running on the processor
# make compile_all                <- compile every program at once (in parallel with -j)

# ---- Dump Files ---- #
# make <my_program>.dump  <- disassembles <my_program>.mem into .dump_x and .dump_abi RISC-V assembly files
# make *.debug.dump       <- for a .c program, creates dump files after compiling with a debug flag
# make programs/<my_program>.dump_x    <- numeric dump files use x0-x31 as register names
# make programs/<my_program>.dump_abi  <- abi dump files use the abi register names (sp, a0, etc.)
# make dump_all  <- create all dump files at once (in parallel with -j)

# ---- Verdi ---- #
# make <my_program>.verdi     <- run a program in verdi via simv
# make <my_program>.syn.verdi <- run a program in verdi via syn_simv

# ---- Visual Debugger ---- #
# make <my_program>.vis  <- run a program on the project 3 vtuber visual debugger!
# make vis_simv          <- compile the vtuber executable from VTUBER and SOURCES

# ---- Cleanup ---- #
# make clean            <- remove per-run files and compiled executable files
# make nuke             <- remove all files created from make rules
# make clean_run_files  <- remove per-run output files
# make clean_exe        <- remove compiled executable files
# make clean_synth      <- remove generated synthesis files
# make clean_output     <- remove the entire output/ directory
# make clean_programs   <- remove program memory and dump files

# Credits:
# VeriSimpleV was adapted by Jielun Tan for RISC-V from the original 470 VeriSimple Alpha language processor
# however I cannot find the original authors or the major editors of the project :/
# so to everyone I can't credit: thank you!
# the current layout of the Makefile was made by Ian Wrzesinski in 2023
# VeriSimpleV has also been edited by at least:
# Nevil Pooniwala, Xueyang Liu, Cassie Jones, James Connolly

######################################################
# ---- Compilation Commands and Other Variables ---- #
######################################################

# these are various build flags for different parts of the makefile, VCS and LIB should be
# familiar, but there are new variables for supporting the compilation of assembly and C
# source programs into riscv machine code files to be loaded into the processor's memory

# don't be afraid to change these, but be diligent about testing changes and using git commits
# there should be no need to change anything for project 3

# this is a global clock period variable used in the tcl script and referenced in testbenches
export CLOCK_PERIOD = 1000.0

# Path variables
export RISCV32_HOME = /homes/user/fac/tk3070/tmp/riscv-gcc/riscv-32/bin
export ELF2HEX_HOME = /homes/user/fac/tk3070/tmp/elf2hex-build/bin
#export PATH += :/homes/user/fac/tk3070/tmp/riscv-gcc/riscv-32/bin/:/homes/user/fac/tk3070/tmp/elf2hex-build/bin/
ifeq (, $(LD_LIBRARY_PATH))
	export LD_LIBRARY_PATH = /homes/user/fac/tk3070/tmp/test/csee4824-project-3-main/tmp
else
	export LD_LIBRARY_PATH += :/homes/user/fac/tk3070/tmp/test/csee4824-project-3-main/tmp
endif

# the Verilog Compiler command and arguments
VCS = SW_VCS=2020.12-SP2-1 vcs -CFLAGS "-I /homes/user/fac/tk3070/conda/include" -sverilog +vc -Mupdate -line -full64 -kdb -lca -nc \
      -debug_access+all+reverse $(VCS_BAD_WARNINGS) +define+CLOCK_PERIOD=$(CLOCK_PERIOD)ps
# a SYNTH define is added when compiling for synthesis that can be used in testbenches

# remove certain warnings that generate MB of text but can be safely ignored
VCS_BAD_WARNINGS = +warn=noTFIPC +warn=noDEBUG_DEP +warn=noENUMASSIGN

# Verdi executable setup
export VERDI_HOME = /tools/synopsys/verdi/verdi/U-2023.03-SP2-1
VERDI_EXE = $(VERDI_HOME)/bin/verdi

# a reference library of standard structural cells that we link against when synthesizing
LIB = $(wildcard /homes/user/fac/tk3070/tmp/synthesis/OpenROAD-flow-scripts/flow/platforms/asap7/work_around_yosys/asap7sc7p5t*.v)

# the CSEE 4824 synthesis script
TCL_SCRIPT = synth/csee4824_synth.tcl

# Set the shell's pipefail option: causes return values through pipes to match the last non-zero value
# (useful for, i.e. piping to `tee`)
SHELL := $(SHELL) -o pipefail

# The following are new in project 3:

# you might need to update these build flags for project 4, but make sure you know what they do:
# https://gcc.gnu.org/onlinedocs/gcc/RISC-V-Options.html
CFLAGS     = -mno-relax -march=rv32im -mabi=ilp32 -nostartfiles -std=gnu11 -mstrict-align -mno-div
# adjust the optimization if you want programs to run faster; this may obfuscate/change their instructions
OFLAGS     = -O0
ASFLAGS    = -mno-relax -march=rv32im -mabi=ilp32 -nostartfiles -Wno-main -mstrict-align
OBJFLAGS   = -SD -M no-aliases
OBJCFLAGS  = --set-section-flags .bss=contents,alloc,readonly
OBJDFLAGS  = -SD -M numeric,no-aliases
DEBUG_FLAG = -g

# this is our RISC-V compiler toolchain
# NOTE: you can use a local riscv install to compile programs by setting CAEN to 0
CAEN = 0
ifeq (1, $(CAEN))
    GCC     = riscv gcc
    OBJCOPY = riscv objcopy
    OBJDUMP = riscv objdump
    AS      = riscv as
    ELF2HEX = riscv elf2hex
else
    GCC     = $(RISCV32_HOME)/riscv32-unknown-elf-gcc
    OBJCOPY = $(RISCV32_HOME)/riscv32-unknown-elf-objcopy
    OBJDUMP = $(RISCV32_HOME)/riscv32-unknown-elf-objdump
    AS      = $(RISCV32_HOME)/riscv32-unknown-elf-as
    ELF2HEX = $(ELF2HEX_HOME)/elf2hex
endif

GREP = grep -E --color=auto

################################
# ---- Module Testbenches ---- #
################################

# This section facilitates creating individual testbenches for your modules
# By using the following naming convention:
# - the source file: 'verilog/rob.sv'
# - declares a module: 'rob'
# - with testbench in: 'test/rob_test.sv'
# - added to TESTED_MODULES as: 'rob'
# - with dependencies: 'rob.simv', 'rob.cov', and 'synth/rob.vg'

# TODO: add more modules here
TESTED_MODULES = mult rob

MODULE = pipeline

# TODO: add verilog module dependencies here:
# (do not include header files)
# Helper function:
DEPS = $(1).simv $(1).cov synth/$(1).vg

MULT_DEPS = verilog/mult_stage.sv
$(call DEPS,mult): $(MULT_DEPS)

# No dependencies for the rob (TODO: add any you create)
ROB_DEPS =
$(call DEPS,rob): $(ROB_DEPS)

# This allows you to use the following make targets:
# make <module>.pass   <- greps for "@@@ Passed" or "@@@ Incorrect" in the output
# make <module>.out    <- run the testbench (via <module>.simv)
# make <module>.simv   <- compile the testbench executable
# make <module>.verdi  <- run in verdi (via <module>.simv)
# make <module>.syn.pass   <- greps for "@@@ Passed" or "@@@ Incorrect" in the output
# make <module>.syn.out    <- run the synthesized module on the testbench
# make <module>.syn.simv   <- compile the synthesized module with the testbench
# make synth/<module>.vg   <- synthesize the module
# make <module>.syn.verdi  <- run in verdi (via <module>.syn.simv)

# We have also added targets for checking testbench coverage:
# make <module>.coverage    <- print the coverage hierarchy report to the terminal
# make <module>.cov.verdi   <- open the coverage report in verdi
# make <module>.cov         <- compiles a coverage executable for the module and testbench
# make <module>.cov.vdb     <- runs the executable and creates the <module>.cov.vdb directory
# make <module>_cov_report  <- run urg to create human readable coverage reports

# The following Makefile targets heavily use pattern substitution and static pattern rules
# See these links if you're curious about how these work:
# - https://www.gnu.org/software/make/manual/html_node/Text-Functions.html
# - https://www.gnu.org/software/make/manual/html_node/Static-Usage.html
# - https://www.gnu.org/software/make/manual/html_node/Automatic-Variables.html

# You shouldn't need to change things below here

# ---- Running ---- #

# run compiled executables ('make %.out' is linked to 'make output/%.out' further below)
# using this syntax avoids overlapping with the 'make <my_program>.out' targets
$(TESTED_MODULES:%=output/%.out) $(TESTED_MODULES:%=output/%.syn.out): output/%.out: %.simv | output
	@$(call PRINT_COLOR, 5, running $<)
	./$< | tee $@
	@$(call PRINT_COLOR, 2, output is in $@)

# print in green or red passed or failed (must $display @@@ Passed or @@@ Incorrect)
%.pass: output/%.out
	@GREP_COLOR="01;31" $(GREP) -i '@@@ ?Incorrect' $< || \
	GREP_COLOR="01;32" $(GREP) -i '@@@ ?Passed' $<
.PHONY: %.pass

# run many types of things in verdi
%.verdi: %.simv
	@$(call PRINT_COLOR, 5, running $< with verdi )
	./$< -gui=$(VERDI_EXE)
.PHONY: %.verdi

# ---- Compiling Verilog ---- #

# the normal simulation executable will run your testbench on simulated modules
$(TESTED_MODULES:=.simv): %.simv: test/%_test.sv verilog/%.sv $(HEADERS)
	@$(call PRINT_COLOR, 5, compiling the simulation executable $@)
	@$(call PRINT_COLOR, 3, NOTE: if this is slow to startup: run '"module load vcs verdi synopsys-synth"')
	$(VCS) $(filter-out $(HEADERS),$^) -o $@
	@$(call PRINT_COLOR, 6, finished compiling $@)

# this also generates many other files, see the tcl script's introduction for info on each of them
synth/%.vg: verilog/%.sv | $(TCL_SCRIPT) $(HEADERS)
	@$(call PRINT_COLOR, 5, synthesizing the $* module)
	@$(call PRINT_COLOR, 3, this might take a while...)
	@$(call PRINT_COLOR, 3, NOTE: if this is slow to startup: run '"module load vcs verdi synopsys-synth"')
	# pipefail causes the command to exit on failure even though it's piping to tee
	set -o pipefail; cd synth && \
	MODULE=$* SOURCES="$(filter-out $(TCL_SCRIPT) $(HEADERS),$^)" \
	dc_shell-t -f $(notdir $(TCL_SCRIPT)) | tee $*_synth.out
	@$(call PRINT_COLOR, 6, finished synthesizing $@)

# the synthesis executable runs your testbench on the synthesized versions of your modules
$(TESTED_MODULES:=.syn.simv): %.syn.simv: test/%_test.sv synth/%.vg $(HEADERS)
	@$(call PRINT_COLOR, 5, compiling the synthesis executable $@)
	$(VCS) +define+SYNTH $(filter-out $(HEADERS),$^) $(LIB) -o $@
	@$(call PRINT_COLOR, 6, finished compiling $@)

# ---- Coverage targets ---- #

# This section adds targets to run module testbenches with coverage output

# VCS argument to both build and run for coverage
VCS_COVG = -cm line+fsm+cond+tgl+branch

$(TESTED_MODULES:=.cov): %.cov: test/%_test.sv verilog/%.sv $(HEADERS)
	@$(call PRINT_COLOR, 5, compiling the coverage executable $@)
	@$(call PRINT_COLOR, 3, NOTE: if this is slow to startup: run '"module load vcs verdi synopsys-synth"')
	$(VCS) $(VCS_COVG) $(filter-out $(HEADERS),$^) -o $@
	@$(call PRINT_COLOR, 6, finished compiling $@)

# Run the testbench to produce a *.vdb directory with coverage info
$(TESTED_MODULES:%=output/%.cov.out): output/%.cov.out: %.cov | output
	@$(call PRINT_COLOR, 5, running $<)
	./$< $(VCS_COVG) | tee output/$*.cov.out
	@$(call PRINT_COLOR, 2, created coverage dir $<.vdb and saved output to $@)

# A layer of indirection is needed to avoid re-making??? :/
%.cov.vdb: output/%.cov.out ;

# Use urg to generate human-readable report files in text mode (alternative is html)
$(TESTED_MODULES:=_cov_report): %_cov_report: %.cov.vdb | output
	@$(call PRINT_COLOR, 5, outputting coverage report in $@)
	urg -format text -dir $< -report $@
	@$(call PRINT_COLOR, 2, coverage report is in $@)

# view the coverage hierarchy report
$(TESTED_MODULES:=.coverage): %.coverage: %_cov_report
	@$(call PRINT_COLOR, 2, printing coverage hierarchy - see $< for more)
	cat $</hierarchy.txt

# open the coverage info in verdi
$(TESTED_MODULES:=.cov.verdi): %.cov.verdi: %.cov.vdb
	@$(call PRINT_COLOR, 5, opening verdi for $* coverage)
	./$(<:.vdb=) -gui=$(VERDI_EXE) -cov -covdir $<

.PHONY: %.coverage %.cov.verdi

####################################
# ---- Executable Compilation ---- #
####################################

# NOTE: the executables are not the only things you need to compile
# you must also create a programs/*.mem file for each program you run
# which will be loaded into test/mem.sv by the testbench on startup
# To run a program on simv or syn_simv, see the program execution section
# This is done automatically with 'make <my_program>.out'

HEADERS = verilog/sys_defs.svh \
          verilog/ISA.svh

TESTBENCH = test/pipeline_test.sv \
            test/pipeline_print.c \
            test/mem.sv

# you could simplify this line with $(wildcard verilog/*.sv) - but the manual way is more explicit
SOURCES = verilog/pipeline.sv \
          verilog/regfile.sv \
          verilog/icache.sv \
          verilog/mult.sv \
          verilog/mult_stage.sv \

SYNTH_FILES = synth/pipeline.vg

# the normal simulation executable will run your testbench on the original modules
simv: $(TESTBENCH) $(SOURCES) $(HEADERS)
	@$(call PRINT_COLOR, 5, compiling the simulation executable $@)
	@$(call PRINT_COLOR, 3, NOTE: if this is slow to startup: run '"module load vcs verdi synopsys-synth"')
	$(VCS) $(filter-out $(HEADERS),$^) -o $@
	@$(call PRINT_COLOR, 6, finished compiling $@)

# NOTE: this has been changed to avoid conflicting with the module %.vg rule
# this also generates many other files, see the tcl script's introduction for info on each of them
synth/pipeline.vg: $(SOURCES) $(TCL_SCRIPT) $(HEADERS)
	@$(call PRINT_COLOR, 5, synthesizing the pipeline)
	@$(call PRINT_COLOR, 3, this might take a while...)
	@$(call PRINT_COLOR, 3, NOTE: if this is slow to startup: run '"module load vcs verdi synopsys-synth"')
	# pipefail causes the command to exit on failure even though it's piping to tee
	set -o pipefail; cd synth && \
	MODULE=pipeline SOURCES="$(SOURCES)" \
	dc_shell-t -f $(notdir $(TCL_SCRIPT)) | tee pipeline_synth.out
	@$(call PRINT_COLOR, 6, finished synthesizing $@)

# the synthesis executable runs your testbench on the synthesized versions of your modules
syn_simv: $(TESTBENCH) $(SYNTH_FILES) $(HEADERS)
	@$(call PRINT_COLOR, 5, compiling the synthesis executable $@)
	$(VCS) +define+SYNTH $(filter-out $(HEADERS),$^) $(LIB) -o $@
	@$(call PRINT_COLOR, 6, finished compiling $@)

# a phony target to view the slack in the *.rep synthesis report file
slack:
	$(GREP) "slack" synth/*.rep
.PHONY: slack

########################################
# ---- Program Memory Compilation ---- #
########################################

# this section will compile programs into .mem files to be loaded into memory
# you start with either an assembly or C program in the programs/ directory
# those compile into a .elf link file via the riscv assembler or compiler
# then that link file is converted to a .mem hex file

# find the test program files and separate them based on suffix of .s or .c
# filter out files that aren't themselves programs
NON_PROGRAMS = $(CRT)
ASSEMBLY = $(filter-out $(NON_PROGRAMS),$(wildcard programs/*.s))
C_CODE   = $(filter-out $(NON_PROGRAMS),$(wildcard programs/*.c))

# concatenate ASSEMBLY and C_CODE to list every program
PROGRAMS = $(ASSEMBLY:%.s=%) $(C_CODE:%.c=%)

# NOTE: this is Make's pattern substitution syntax
# see: https://www.gnu.org/software/make/manual/html_node/Text-Functions.html#Text-Functions
# this reads as: $(var:pattern=replacement)
# a percent sign '%' in pattern is as a wildcard, and can be reused in the replacement
# if you don't include the percent it automatically attempts to replace just the suffix of the input

# C and assembly compilation files. These link and setup the runtime for the programs
CRT        = programs/crt.s
LINKERS    = programs/linker.lds
ASLINKERS  = programs/aslinker.lds

# make elf files from assembly code
%.elf: %.s $(ASLINKERS)
	@$(call PRINT_COLOR, 5, compiling assembly file $<)
	$(GCC) $(ASFLAGS) $< -T $(ASLINKERS) -o $@

# make elf files from C source code
%.elf: %.c $(CRT) $(LINKERS)
	@$(call PRINT_COLOR, 5, compiling C code file $<)
	$(GCC) $(CFLAGS) $(OFLAGS) $(CRT) $< -T $(LINKERS) -o $@

# C programs can also be compiled in debug mode, this is solely meant for use in the .dump files below
%.debug.elf: %.c $(CRT) $(LINKERS)
	@$(call PRINT_COLOR, 5, compiling debug C code file $<)
	$(GCC) $(CFLAGS) $(OFLAGS) $(CRT) $< -T $(LINKERS) -o $@
	$(GCC) $(DEBUG_FLAG) $(CFLAGS) $(OFLAGS) $(CRT) $< -T $(LINKERS) -o $@

# declare the .elf files as intermediate files.
# Make will automatically rm intermediate files after they're used in a recipe
# and it won't remake them until their sources are updated or they're needed again
.INTERMEDIATE: %.elf

# turn any elf file into a hex memory file ready for the testbench
%.mem: %.elf
	$(ELF2HEX) --bit-width 64 --input $< --output $@
	@$(call PRINT_COLOR, 6, created memory file $@)
	@$(call PRINT_COLOR, 3, NOTE: to see RISC-V assembly run: '"make $*.dump"')
	@$(call PRINT_COLOR, 3, for \*.c sources also try: '"make $*.debug.dump"')

# compile all programs in one command (use 'make -j' to run multithreaded)
compile_all: $(PROGRAMS:=.mem)
.PHONY: compile_all

########################
# ---- Dump Files ---- #
########################

# when debugging a program, the dump files will show you the disassembled RISC-V
# assembly code that your processor is actually running

# this creates the <my_program>.debug.elf targets, which can be used in: 'make <my_program>.debug.dump_*'
# these are useful for the C sources because the debug flag makes the assembly more understandable
# because it includes some of the original C operations and function/variable names

DUMP_PROGRAMS = $(ASSEMBLY:.c=) $(C_CODE:.c=.debug)

# 'make <my_program>.dump' will create both files at once!
./%.dump: programs/%.dump_x programs/%.dump_abi ;
.PHONY: ./%.dump
# Tell tell Make to treat the .dump_* files as "precious" and not to rm them as intermediaries to %.dump
.PRECIOUS: %.dump_x %.dump_abi

# use the numberic x0-x31 register names
%.dump_x: %.elf
	@$(call PRINT_COLOR, 5, disassembling $<)
	$(OBJDUMP) $(OBJDFLAGS) $< > $@
	@$(call PRINT_COLOR, 6, created numeric dump file $@)

# use the Application Binary Interface register names (sp, a0, etc.)
%.dump_abi: %.elf
	@$(call PRINT_COLOR, 5, disassembling $<)
	$(OBJDUMP) $(OBJFLAGS) $< > $@
	@$(call PRINT_COLOR, 6, created abi dump file $@)

# create all dump files in one command (use 'make -j' to run multithreaded)
dump_all: $(DUMP_PROGRAMS:=.dump_x) $(DUMP_PROGRAMS:=.dump_abi)
.PHONY: dump_all

###############################
# ---- Program Execution ---- #
###############################

# run one of the executables (simv/syn_simv) using the chosen program
# e.g. 'make sampler.out' does the following from a clean directory:
#   1. compiles simv
#   2. compiles programs/sampler.s into its .elf and then .mem files (in programs/)
#   3. runs ./simv +MEMORY=programs/sampler.mem +WRITEBACK=output/sampler.wb +PIPELINE=output/sampler.ppln > output/sampler.out
#   4. which creates the sampler.out, sampler.wb, and sampler.ppln files in output/
# the same can be done for synthesis by doing 'make sampler.syn.out'
# which will also create .syn.wb and .syn.ppln files in output/

# targets built in the 'output/' directory should create output/ if it doesn't exist
# (it's deleted entirely by 'make nuke')
# NOTE: place it after the pipe "|" as an order-only pre-requisite
output:
	mkdir -p output

OUTPUTS = $(PROGRAMS:programs/%=output/%)

# run a program and produce output files
$(OUTPUTS:=.out): output/%.out: programs/%.mem simv | output
	@$(call PRINT_COLOR, 5, running simv on $<)
	./simv +MEMORY=$< +WRITEBACK=$(@D)/$*.wb +PIPELINE=$(@D)/$*.ppln > $@
	@$(call PRINT_COLOR, 6, finished running simv on $<)
	@$(call PRINT_COLOR, 2, output is in $@ $(@D)/$*.wb and $(@D)/$*.ppln)
# NOTE: this uses a 'static pattern rule' to match a list of known targets to a pattern
# and then generates the correct rule based on the pattern, where % and $* match
# so for the target 'output/sampler.out' the % matches 'sampler' and depends on programs/sampler.mem
# see: https://www.gnu.org/software/make/manual/html_node/Static-Usage.html
# $(@D) is an automatic variable for the directory of the target, in this case, 'output'

# this does the same as simv, but adds .syn to the output files and compiles syn_simv instead
# run synthesis with: 'make <my_program>.syn.out'
$(OUTPUTS:=.syn.out): output/%.syn.out: programs/%.mem syn_simv | output
	@$(call PRINT_COLOR, 5, running syn_simv on $<)
	@$(call PRINT_COLOR, 3, this might take a while...)
	./syn_simv +MEMORY=$< +WRITEBACK=$(@D)/$*.syn.wb +PIPELINE=$(@D)/$*.syn.ppln > $@
	@$(call PRINT_COLOR, 6, finished running syn_simv on $<)
	@$(call PRINT_COLOR, 2, output is in $@ $(@D)/$*.syn.wb and $(@D)/$*.syn.ppln)

# Allow us to type 'make <my_program>.out' instead of 'make output/<my_program>.out'
./%.out: output/%.out ;
.PHONY: ./%.out

# Declare that creating a %.out file also creates both %.wb and %.ppln files
%.wb %.ppln: %.out ;

# run all programs in one command (use 'make -j' to run multithreaded)
simulate_all: simv compile_all $(OUTPUTS:=.out)
simulate_all_syn: syn_simv compile_all $(OUTPUTS:=.syn.out)
.PHONY: simulate_all simulate_all_syn

###################
# ---- Verdi ---- #
###################

# run verdi on a program with: 'make <my_program>.verdi' or 'make <my_program>.syn.verdi'

# this creates a directory verdi will use if it doesn't exist yet
verdi_dir:
	mkdir -p /workdir/$${USER}_csee4824
.PHONY: verdi_dir

novas.rc: initialnovas.rc
	sed s/UNIQNAME/$$USER/ initialnovas.rc > novas.rc

%.verdi: programs/%.mem simv novas.rc verdi_dir
	./simv -gui=$(VERDI_EXE) +MEMORY=$< +WRITEBACK=/dev/null +PIPELINE=/dev/null

%.syn.verdi: programs/%.mem syn_simv novas.rc verdi_dir
	./syn_simv -gui=$(VERDI_EXE) +MEMORY=$< +WRITEBACK=/dev/null +PIPELINE=/dev/null

.PHONY: %.verdi

#############################
# ---- Visual Debugger ---- #
#############################

# this is the visual debugger for project 3, an extremely helpful tool, try it out!
# compile and run the visual debugger on a program with:
# 'make <my_program>.vis'

# Don't ask me why we spell VisUal TestBenchER like this...
VTUBER = test/vtuber_test.sv \
         test/vtuber.cpp \
		 test/mem.sv

VISFLAGS = -lncurses

vis_simv: $(HEADERS) $(VTUBER) $(SOURCES)
	@$(call PRINT_COLOR, 5, compiling visual debugger testbench)
	$(VCS) $(VISFLAGS) $^ -o vis_simv
	@$(call PRINT_COLOR, 6, finished compiling visual debugger testbench)

%.vis: programs/%.mem vis_simv
	./vis_simv +MEMORY=$<
	@$(call PRINT_COLOR, 6, Fullscreen your terminal for the best VTUBER experience!)
.PHONY: %.vis

#####################
# ---- Cleanup ---- #
#####################

# You should only clean your directory if you think something has built incorrectly
# or you want to prepare a clean directory for e.g. git (first check your .gitignore).
# Please avoid cleaning before every build. The point of a makefile is to
# automatically determine which targets have dependencies that are modified,
# and to re-build only those as needed; avoiding re-building everything everytime.

# 'make clean' removes build/output files, 'make nuke' removes all generated files
# 'make clean' does not remove .mem or .dump files
# clean_* commands remove certain groups of files

clean: clean_exe clean_run_files
	@$(call PRINT_COLOR, 6, note: clean is split into multiple commands you can call separately: $^)

# removes all extra synthesis files and the entire output directory
# use cautiously, this can cause hours of recompiling in project 4
nuke: clean clean_output clean_synth clean_programs
	@$(call PRINT_COLOR, 6, note: nuke is split into multiple commands you can call separately: $^)

clean_exe:
	@$(call PRINT_COLOR, 3, removing compiled executable files)
	rm -rf *simv *.daidir csrc *.key   # created by simv/syn_simv/vis_simv
	rm -rf *.vdb *.vpd vc_hdrs.h       # created by simv/syn_simv/vis_simv
	rm -rf *.cov *cov_report cm.log    # coverage files
	rm -rf verdi* novas* *fsdb*        # verdi files
	rm -rf dve* inter.vpd DVEfiles     # old DVE debugger

clean_run_files:
	@$(call PRINT_COLOR, 3, removing per-run outputs)
	rm -rf output/*.out output/*.wb output/*.ppln

clean_synth:
	@$(call PRINT_COLOR, 1, removing synthesis files)
	cd synth && rm -rf *.vg *_svsim.sv *.res *.rep *.ddc *.chk *.syn *.out *.db *.svf *.mr *.pvl command.log

clean_output:
	@$(call PRINT_COLOR, 1, removing entire output directory)
	rm -rf output/

clean_programs:
	@$(call PRINT_COLOR, 3, removing program memory files)
	rm -rf programs/*.mem
	@$(call PRINT_COLOR, 3, removing dump files)
	rm -rf programs/*.dump*

.PHONY: clean nuke clean_%

######################
# ---- Printing ---- #
######################

# this is a GNU Make function with two arguments: PRINT_COLOR(color: number, msg: string)
# it does all the color printing throughout the makefile
PRINT_COLOR = if [ -t 0 ]; then tput setaf $(1) ; fi; echo $(2); if [ -t 0 ]; then tput sgr0; fi
# colors: 0:black, 1:red, 2:green, 3:yellow, 4:blue, 5:magenta, 6:cyan, 7:white
# other numbers are valid, but aren't specified in the tput man page

# Make functions are called like this:
# $(call PRINT_COLOR,3,Hello World!)
# NOTE: adding '@' to the start of a line avoids printing the command itself, only the output
