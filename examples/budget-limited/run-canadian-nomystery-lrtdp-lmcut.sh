#!/bin/sh

python ../../src/fast-downward.py --alias lrtdp-lmcut canadian-nomystery-120/p05-problem.pddl/problem.pddl --preprocess-options --nora --search-options --budget canadian-nomystery-120/p05-problem.pddl/problem.budget
rm -f output
rm -f output.sas

