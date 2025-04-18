#!/bin/bash

# Move all *.syn.simv.daidir to post_syn/
find . -maxdepth 1 -type d -name "*.simv.daidir" -exec mv -v {} simv_testing/ \;
find . -maxdepth 1 -type f -name "*.simv" -exec mv -v {} simv_testing/ \;

echo "✅ Done! Organized simv.daidir folders into simv_testing/{pre_syn,post_syn}"
