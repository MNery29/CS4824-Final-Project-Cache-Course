
# a quick Tcl reference:
# - Tcl = Tool command language
#   a simple and extensible command language, designed to easily add custom vendor commands
# - every line represents a command, use a semicolon to separate commands on one line
# - comment lines start with a #, and there are no inline comments (unless you use a semicolon...)
# - command arguments are separated by spaces
    mycommand arg1 arg2 arg3
# - everything is a string
# - puts is the print command
# - set assigns variables, $ accesses them
    set myvar Hello!
    puts $myvar
# - to group words with spaces as one string, use "" or {}
    set a "Hello, World!" ;# "" allows variable and bracket substitution
    set b {Hello, World!} ;# {} gives raw strings without variable substitution
# - also common to reference a variable inside curly braces:
#   ${x} equals $x, but allows concatenation:
    set x 5; set a ${x}point${x}; set b $xpoint$x ; #(no variable "xpoint"!)
# - use a backslash to quote special characters: set a \{\$hi\}\;\#hello
#   or to continue a long line as the same command
#   you can even continue comments with backslashes \
    wait what?! ;_;
# - expr command does math and equality checking (true is 1, false is 0)
#   understands +, *, /, %, ==, <, &, &&, pow(x,y) abs(x), round(x), x?a:b, sin(x), exp(x), etc.
# - square brackets [] do command substitution:
    set a [expr 5 + abs(-6)]
#   gives 11
# - getenv command reads variables from the environment, useful for Makefile coordination
    set headers [getenv HEADERS]
# - syntax for a while command:
    while test body
#   generally written as:
    set x 0
    while {$x < 10} {incr x}
#   or: (note the open curly brace { must be on the same line)
    while {$x < 10} {
      incr x
    }
#   the braces are just getting the raw strings of the commands! (and the test is evaluated as in expr)
# - syntax for if statement:
    if {$x == 1} {
      puts "x is one"
    } elseif {$x == 2} {
      puts "x is two"
    } else {
      puts "x is not one or two"
    }
# - the help and man commands are implemented for dc_shell specific commands
#   the -verbose (-v) flag is almost always a good idea
    help create_clock; # try this one first for the comedic effect
    help -verbose create_clock
    help -v set_clock_uncertainty
    man analyze; # run inside the dc_shell terminal
#   recommended command line man usage: dc_shell -x "man analyze; quit" > oof.txt; less oof.txt
# - and thus ends the quick reference!

# links!
# html book I got some of this from:  http://www.beedub.com/book/2nd/tclintro.doc.html
# Using Tcl With Synopsys Tools pdf: https://www.edaboard.com/attachments/tcl_scripting_language-pdf.91704/
# Synopsys Documents (note: design compiler user guide): https://github.com/hyf6661669/Synopsys-Documents
# for non-dc_shell commands: the tcl command man page! https://www.tcl.tk/man/tcl/TclCmd/contents.html
