#!/bin/sh

python ../../src/fast-downward.py --alias acyclic-vi blocksworld-ipc6-100/p01-c0-C0-g1-n5-problem.pddl/problem.pddl --preprocess-options --nora --search-options --budget blocksworld-ipc6-100/p01-c0-C0-g1-n5-problem.pddl/problem.budget
rm -f output
rm -f output.sas

