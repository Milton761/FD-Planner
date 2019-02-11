#!/bin/sh

python ../../src/fast-downward.py --alias ao-ms100k zenotravel-ipc6-120/p01-c4-p2-a2-s3846.pddl/problem.pddl --preprocess-options --nora --search-options --budget zenotravel-ipc6-120/p01-c4-p2-a2-s3846.pddl/problem.budget
rm -f output
rm -f output.sas

