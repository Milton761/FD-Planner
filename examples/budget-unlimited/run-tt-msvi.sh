#!/bin/sh

python ../../src/fast-downward.py --alias msvi triangle-tireworld-ipc6/p20.pddl/problem.pddl --preprocess-options --nora
rm -f output
rm -f output.sas

