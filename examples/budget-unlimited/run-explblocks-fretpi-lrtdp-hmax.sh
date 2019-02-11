#!/bin/sh

python ../../src/fast-downward.py --alias fretpi-lrtdp-hmax exploding-blocksworld-ipc6/p10-n10-N12-s10.pddl/problem.pddl --preprocess-options --nora
rm -f output
rm -f output.sas

