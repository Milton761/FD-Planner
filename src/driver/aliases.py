# -*- coding: utf-8 -*-
from __future__ import print_function

import os

from .util import DRIVER_DIR


PORTFOLIO_DIR = os.path.join(DRIVER_DIR, "portfolios")

################################################################################
################################################################################
# BEGIN-ALIASES
################################################################################
################################################################################

ALIASES = {}

################################################################################
# GSSP configurations:
################################################################################

ALIASES["vi"] = ["--search", "tvi(state_space=factored(prune=blind()), epsilon=0.00005)"]
ALIASES["msvi"] = ["--search", "tvi(state_space=state_space=mas(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)]), epsilon=0.00005)"]
ALIASES["vi-hmax"] = ["--search", "tvi(state_space=factored(prune=hmax()), epsilon=0.00005)"]
ALIASES["vi-msinf"] = ["--search", "tvi(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)), epsilon=0.00005)"]
ALIASES["vi-ms100k"] = ["--search", "tvi(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=100000,max_states_before_merge=100000,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)), epsilon=0.00005)"]

ALIASES["fretv-lrtdp"] = ["--search", "fret(state_space=factored(prune=blind()), engine=lrtdp(epsconsist=true, oselect=default, tiebreaking=arbitrary, epsilon=0.00005), local=false, epsilon=0.00005, delta=0.0005)"]
ALIASES["ms-fretv-lrtdp"] = ["--search", "fret(state_space=state_space=mas(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)]), engine=lrtdp(epsconsist=true, oselect=default, tiebreaking=arbitrary, epsilon=0.00005), local=false, epsilon=0.00005, delta=0.0005)"]
ALIASES["fretv-lrtdp-hmax"] = ["--search", "fret(state_space=factored(prune=hmax()), engine=lrtdp(epsconsist=true, oselect=default, tiebreaking=arbitrary, epsilon=0.00005), local=false, epsilon=0.00005, delta=0.0005)"]
ALIASES["fretv-lrtdp-msinf"] = ["--search", "fret(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)), engine=lrtdp(epsconsist=true, oselect=default, tiebreaking=arbitrary, epsilon=0.00005), local=false, epsilon=0.00005, delta=0.0005)"]
ALIASES["fretv-lrtdp-ms100k"] = ["--search", "fret(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=100000,max_states_before_merge=100000,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)), engine=lrtdp(epsconsist=true, oselect=default, tiebreaking=arbitrary, epsilon=0.00005), local=false, epsilon=0.00005, delta=0.0005)"]

ALIASES["fretpi-lrtdp"] = ["--search", "fret(state_space=factored(prune=blind()), engine=lrtdp(epsconsist=true, oselect=default, tiebreaking=arbitrary, epsilon=0.00005), local=true, epsilon=0.00005, delta=0.0005)"]
ALIASES["ms-fretpi-lrtdp"] = ["--search", "fret(state_space=state_space=mas(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)]), engine=lrtdp(epsconsist=true, oselect=default, tiebreaking=arbitrary, epsilon=0.00005), local=true, epsilon=0.00005, delta=0.0005)"]
ALIASES["fretpi-lrtdp-hmax"] = ["--search", "fret(state_space=factored(prune=hmax()), engine=lrtdp(epsconsist=true, oselect=default, tiebreaking=arbitrary, epsilon=0.00005), local=true, epsilon=0.00005, delta=0.0005)"]
ALIASES["fretpi-lrtdp-msinf"] = ["--search", "fret(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)), engine=lrtdp(epsconsist=true, oselect=default, tiebreaking=arbitrary, epsilon=0.00005), local=true, epsilon=0.00005, delta=0.0005)"]
ALIASES["fretpi-lrtdp-ms100k"] = ["--search", "fret(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=100000,max_states_before_merge=100000,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)), engine=lrtdp(epsconsist=true, oselect=default, tiebreaking=arbitrary, epsilon=0.00005), local=true, epsilon=0.00005, delta=0.0005)"]

################################################################################
# SSP configurations
################################################################################

ALIASES["lrtdp"] = ["--search", "lrtdp(state_space=factored(prune=blind()), epsconsist=false)"]
ALIASES["mslrtdp"] = ["--search", "lrtdp(state_space=state_space=mas(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)]), epsconsist=false)"]
ALIASES["lrtdp-lmcut"] = ["--search", "lrtdp(state_space=factored(prune=lmcut()), epsconsist=false)"]
ALIASES["lrtdp-msinf"] = ["--search", "lrtdp(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)), epsconsist=false)"]
ALIASES["lrtdp-ms100k"] = ["--search", "lrtdp(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=100000,max_states_before_merge=100000,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)), epsconsist=false)"]

ALIASES["hdp"] = ["--search", "hdfs(state_space=factored(prune=blind()), labeling=true, fw_updates=true, bw_updates=true, inconsistent=true, terminate_trial=false, vi=false, tiebreaking=arbitrary, epsilon=0.00005)"]
ALIASES["mshdp"] = ["--search", "hdfs(state_space=state_space=mas(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)]), labeling=true, fw_updates=true, bw_updates=true, inconsistent=true, terminate_trial=false, vi=false, tiebreaking=arbitrary, epsilon=0.00005)"]
ALIASES["hdp-lmcut"] = ["--search", "hdfs(state_space=factored(prune=hmax()), labeling=true, fw_updates=true, bw_updates=true, inconsistent=true, terminate_trial=false, vi=false, tiebreaking=arbitrary, epsilon=0.00005)"]
ALIASES["hdp-msinf"] = ["--search", "hdfs(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)), labeling=true, fw_updates=true, bw_updates=true, inconsistent=true, terminate_trial=false, vi=false, tiebreaking=arbitrary, epsilon=0.00005)"]
ALIASES["hdp-ms100k"] = ["--search", "hdfs(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=100000,max_states_before_merge=100000,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)), labeling=true, fw_updates=true, bw_updates=true, inconsistent=true, terminate_trial=false, vi=false, tiebreaking=arbitrary, epsilon=0.00005)"]

ALIASES["ilao"] = ["--search", "hdfs(state_space=factored(prune=blind()), labeling=false, fw_updates=true, bw_updates=true, inconsistent=false, terminate_trial=false, vi=true, tiebreaking=arbitrary, epsilon=0.00005)"]
ALIASES["msilao"] = ["--search", "hdfs(state_space=state_space=mas(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)]), labeling=false, fw_updates=true, bw_updates=true, inconsistent=false, terminate_trial=false, vi=true, tiebreaking=arbitrary, epsilon=0.00005)"]
ALIASES["ilao-lmcut"] = ["--search", "hdfs(state_space=factored(prune=hmax()), labeling=false, fw_updates=true, bw_updates=true, inconsistent=false, terminate_trial=false, vi=true, tiebreaking=arbitrary, epsilon=0.00005)"]
ALIASES["ilao-msinf"] = ["--search", "hdfs(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)), labeling=false, fw_updates=true, bw_updates=true, inconsistent=false, terminate_trial=false, vi=true, tiebreaking=arbitrary, epsilon=0.00005)"]
ALIASES["ilao-ms100k"] = ["--search", "hdfs(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=100000,max_states_before_merge=100000,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)), labeling=false, fw_updates=true, bw_updates=true, inconsistent=false, terminate_trial=false, vi=true, tiebreaking=arbitrary, epsilon=0.00005)"]

################################################################################
# Configurations for acyclic state spaces
# NOTE: The following configurations really should be only used in cases where
# the state space is acyclic!
################################################################################

ALIASES["acyclic-vi"] = ["--search", "avi(state_space=factored(prune=blind()))"]
ALIASES["acyclic-msvi"] = ["--search", "avi(state_space=state_space=mas(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)]))"]
ALIASES["acyclic-vi-lmcut"] = ["--search", "avi(state_space=factored(prune=lmcut()))"]
ALIASES["acyclic-vi-msinf"] = ["--search", "avi(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)))"]
ALIASES["acyclic-vi-ms100k"] = ["--search", "avi(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=100000,max_states_before_merge=100000,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)))"]

ALIASES["ao"] = ["--search", "ao(state_space=factored(prune=blind()), oselect=arbitrary, tiebreaking=arbitrary)"]
ALIASES["msao"] = ["--search", "ao(state_space=state_space=mas(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)]), oselect=arbitrary, tiebreaking=arbitrary)"]
ALIASES["ao-lmcut"] = ["--search", "ao(state_space=factored(prune=lmcut()), oselect=arbitrary, tiebreaking=arbitrary)"]
ALIASES["ao-msinf"] = ["--search", "ao(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)), oselect=arbitrary, tiebreaking=arbitrary)"]
ALIASES["ao-ms100k"] = ["--search", "ao(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=100000,max_states_before_merge=100000,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)), oselect=arbitrary, tiebreaking=arbitrary)"]

ALIASES["aol"] = ["--search", "aol(state_space=factored(prune=blind()), tiplist=lifo)"]
ALIASES["msaol"] = ["--search", "aol(state_space=state_space=mas(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)]))"]
ALIASES["aol-lmcut"] = ["--search", "aol(state_space=factored(prune=lmcut()), tiplist=lifo)"]
ALIASES["aol-msinf"] = ["--search", "aol(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=2147483646,max_states_before_merge=2147483646,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)), tiplist=lifo)"]
ALIASES["aol-ms100k"] = ["--search", "aol(state_space=factored(prune=merge_and_shrink(merge_strategy=merge_linear_ext(criteria=[scc(), goal()], order=level), shrink_strategy=[shrink_bisimulation(max_states=100000,max_states_before_merge=100000,threshold=1)], label_reduction=label_reduction(before_merging=false, before_shrinking=false), shrink_result=true)), tiplist=lifo)"]

################################################################################
################################################################################
# END-ALIASES
################################################################################
################################################################################


PORTFOLIOS = {}
for portfolio in os.listdir(PORTFOLIO_DIR):
    name, ext = os.path.splitext(portfolio)
    assert ext == ".py", portfolio
    PORTFOLIOS[name.replace("_", "-")] = os.path.join(PORTFOLIO_DIR, portfolio)


def show_aliases():
    for alias in sorted(ALIASES.keys() + PORTFOLIOS.keys()):
        print(alias)


def set_options_for_alias(alias_name, args):
    """
    If alias_name is an alias for a configuration, set args.search_options
    to the corresponding command-line arguments. If it is an alias for a
    portfolio, set args.portfolio to the path to the portfolio file.
    Otherwise raise KeyError.
    """
    assert not args.portfolio

    if alias_name in ALIASES:
        backup = []
        if hasattr(args, 'search_options'):
            backup = args.search_options
        args.search_options = [x.replace(" ", "").replace("\n", "")
                               for x in ALIASES[alias_name]] + backup
    elif alias_name in PORTFOLIOS:
        args.portfolio = PORTFOLIOS[alias_name]
    else:
        raise KeyError(alias_name)
