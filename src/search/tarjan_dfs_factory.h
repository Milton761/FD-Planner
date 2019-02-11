
#ifndef TARJAN_DFS_FACTORY_H
#define TARJAN_DFS_FACTORY_H

#include "prob_search_engine_factory.h"

#include "tarjan_dfs.h"

#include "optimization_criteria.h"
#include "action_tie_breaking.h"
#include "value_initializer.h"
#include "value_reporter.h"

#include "prob_search_node_info.h"
#include "prob_search_space.h"

#include "option_parser.h"
#include "plugin.h"

#include "utilities.h"

#include <iostream>
#include <vector>
#include <string>

namespace prob_search_engine_factory
{
extern Plugin<SearchEngine> hdfs;
}

namespace tarjan_dfs_factory
{

void add_options_to_parser(OptionParser &parser);
void copy_options_to_parser(const Options &opts, OptionParser &parser);
void consistency_check(const Options &opts);

template<template<typename> class InfoWrapper, typename SStateInfo, template<template<typename> class, typename> class StateWrapper, template<typename> class SState, template<template<typename, typename> class, typename , template<typename> class> class SSearchSpace, typename ValueInitializer, typename ValueReporter, template<typename> class Callback, typename TTieBreaking, typename CheckSolved, typename VLookup>
struct TarjanMaxP {
    struct StateInfo : InfoWrapper<TarjanStateInfo<SStateInfo> > {
        StateInfo(int sid, int budget) : InfoWrapper<TarjanStateInfo<SStateInfo> >(sid,
                    budget) {}
    };
    template<typename Info> struct _State : public TarjanState<SState, Info> {
        _State(size_t a, Info &b, unsigned u) : TarjanState<SState, Info>(a, b, u) {}
    };
    template<typename Info> struct State : public StateWrapper<_State, Info> {
        State(size_t a, Info &b, unsigned u) : StateWrapper<_State, Info>(a, b, u) {}
    };
    template<template<typename, typename> class A, typename B, template<typename> class C>
    struct SearchSpace : public SSearchSpace<A, B, C> {
        SearchSpace(A<C<B>, typename SSearchSpace<A, B, C>::StateIDLookup>
                    &state_space) : SSearchSpace<A, B, C>(state_space) {}
    };

    typedef ValueInitializer Vinit;
    typedef ValueReporter Vreport;

    using TieBreaking = TTieBreaking;

    template<template<typename, typename> class StateSpace>
    SearchEngine *create(Options &opts) const {
        return this->create<StateInfo, State, SearchSpace, StateSpace, Vinit, Vreport, TieBreaking>
               (opts);
    }

    template<typename t_info, template<typename> class t_state, template<template<typename, typename> class, typename, template<typename> class> class t_search_space, template<typename, typename> class t_state_space, typename t_init, typename t_report, typename t_tie>
    SearchEngine *create(Options &opts) const {
        opts.set<t_init *>("initializer", new t_init());
        opts.set<t_report *>("reporter", new t_report());

        return new TarjanEngine < t_info,
               t_state,
               t_search_space,
               t_state_space,
               t_init,
               t_report,
               Callback,
               t_tie,
               CheckSolved,
               VLookup > (opts);
    }
};

template<template<template<typename, typename> class, typename, template<typename> class> class t_search_space, typename t_vinit, typename t_vreport, template<typename> class Factory>
static SearchEngine *generate_upper(OptionParser &parser)
{
    Options opts = parser.parse();
    if (!parser.dry_run()) {
        consistency_check(opts);
        if (opts.get<bool>("vi")) {
            switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
            case action_tie_breaking::ARBITRARY:
                return Factory < TarjanMaxP <
                       SingleVStateInfo,
                       BasicStateInfo,
                       SingleVState,
                       BasicState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       TieBreakingArbitrary,
                       TVIConverged,
                       OptimisticV > >::create(parser);
#if COMPILE_TIEBREAKING_MINEXPH
            case action_tie_breaking::MINEXPH:
                return Factory < TarjanMaxP <
                       SingleVStateInfo,
                       HStateInfo,
                       SingleVState,
                       HState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       TieBreakingMinExpH,
                       TVIConverged,
                       OptimisticV > >::create(parser);
#endif
#if COMPILE_TIEBREAKING_MINEXPGAP
            case action_tie_breaking::MINEXPGAP:
                std::cerr << "upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_MAXEXGAP
            case action_tie_breaking::MAXEXGAP:
                std::cerr << "upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < TarjanMaxP <
                       SingleVStateInfo,
                       PreferredStateInfo,
                       SingleVState,
                       PreferredState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       TieBreakingPreferredArbitrary,
                       TVIConverged,
                       OptimisticV > >::create(parser);
#endif
            default:
                break;
            }
        } else {
            switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
            case action_tie_breaking::ARBITRARY:
                return Factory < TarjanMaxP <
                       SingleVStateInfo,
                       BasicStateInfo,
                       SingleVState,
                       BasicState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       TieBreakingArbitrary,
                       IsLabeledSolved,
                       OptimisticV > >::create(parser);
#if COMPILE_TIEBREAKING_MINEXPH
            case action_tie_breaking::MINEXPH:
                return Factory < TarjanMaxP <
                       SingleVStateInfo,
                       HStateInfo,
                       SingleVState,
                       HState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       TieBreakingMinExpH,
                       IsLabeledSolved,
                       OptimisticV > >::create(parser);
#endif
#if COMPILE_TIEBREAKING_MINEXPGAP
            case action_tie_breaking::MINEXPGAP:
                std::cerr << "upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_MAXEXGAP
            case action_tie_breaking::MAXEXGAP:
                std::cerr << "upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < TarjanMaxP <
                       SingleVStateInfo,
                       PreferredStateInfo,
                       SingleVState,
                       PreferredState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       TieBreakingPreferredArbitrary,
                       IsLabeledSolved ,
                       OptimisticV > >::create(parser);
#endif
            default:
                break;
            }

        }
    }
    std::cerr << "Unsupported HDFS parameter encountered." << std::endl
              << "Make sure that the constants are set correctly in " << std::endl
              << "  prob_search_engine_factory.h" << std::endl
              << "  tarjan_dfs_factory.h" << std::endl
              << "and recompile it." << std::endl;
    return NULL;
}

template<template<template<typename, typename> class, typename, template<typename> class> class t_search_space, typename t_vinit, typename t_vreport, template<typename> class Factory>
static SearchEngine *generate_dual(OptionParser &parser)
{
    Options opts = parser.parse();
    if (!parser.dry_run()) {
        consistency_check(opts);
        if (opts.get<bool>("vi")) {
            switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
            case action_tie_breaking::ARBITRARY:
                return Factory < TarjanMaxP <
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       TieBreakingArbitrary,
                       TVIConverged,
                       BiOptimisticV > >::create(parser);
#if COMPILE_TIEBREAKING_MINEXPH
            case action_tie_breaking::MINEXPH:
                return Factory < TarjanMaxP <
                       UpperAndLowerVStateInfo,
                       HStateInfo,
                       UpperAndLowerVState,
                       HState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       TieBreakingMinExpH,
                       TVIConverged,
                       BiOptimisticV > >::create(parser);
#endif
#if COMPILE_TIEBREAKING_MINEXPGAP
            case action_tie_breaking::MINEXPGAP:
                return Factory < TarjanMaxP <
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       TieBreakingMinExpGap,
                       TVIConverged,
                       BiOptimisticV > >::create(parser);
#endif
#if COMPILE_TIEBREAKING_MAXEXGAP
            case action_tie_breaking::MAXEXGAP:
                std::cerr << "upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < TarjanMaxP <
                       UpperAndLowerVStateInfo,
                       PreferredStateInfo,
                       UpperAndLowerVState,
                       PreferredState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       TieBreakingPreferredArbitrary,
                       TVIConverged,
                       BiOptimisticV > >::create(parser);
#endif
            default:
                break;
            }
        } else {
            switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
            case action_tie_breaking::ARBITRARY:
                return Factory < TarjanMaxP <
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       TieBreakingArbitrary,
                       IsLabeledSolved,
                       BiOptimisticV > >::create(parser);
#if COMPILE_TIEBREAKING_MINEXPH
            case action_tie_breaking::MINEXPH:
                return Factory < TarjanMaxP <
                       UpperAndLowerVStateInfo,
                       HStateInfo,
                       UpperAndLowerVState,
                       HState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       TieBreakingMinExpH,
                       IsLabeledSolved,
                       BiOptimisticV > >::create(parser);
#endif
#if COMPILE_TIEBREAKING_MINEXPGAP
            case action_tie_breaking::MINEXPGAP:
                return Factory < TarjanMaxP <
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       TieBreakingMinExpGap,
                       IsLabeledSolved,
                       BiOptimisticV > >::create(parser);
#endif
#if COMPILE_TIEBREAKING_MAXEXGAP
            case action_tie_breaking::MAXEXGAP:
                std::cerr << "upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < TarjanMaxP <
                       UpperAndLowerVStateInfo,
                       PreferredStateInfo,
                       UpperAndLowerVState,
                       PreferredState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       TieBreakingPreferredArbitrary,
                       IsLabeledSolved,
                       BiOptimisticV > >::create(parser);
#endif
            default:
                break;
            }

        }
    }
    std::cerr << "Unsupported HDFS parameter encountered." << std::endl
              << "Make sure that the constants are set correctly in " << std::endl
              << "  prob_search_engine_factory.h" << std::endl
              << "  tarjan_dfs_factory.h" << std::endl
              << "and recompile it." << std::endl;
    return NULL;
}

template<template<typename> class Factory>
static SearchEngine *parse_single(OptionParser &parser)
{
    switch (g_optimization_criterion) {
    case OptimizationCriterion::MAXP:
        return generate_upper <
               ProbSearchSpaceMax,
               OptimisticValueInitializer<OptimizationCriterion::MAXP>,
               OptimisticValueReporter<OptimizationCriterion::MAXP>,
               Factory > (parser);
#if COMPILE_OPTIMIZATION_CRITERION_EXPC
    case OptimizationCriterion::EXPC:
        return generate_upper <
               ProbSearchSpaceMin,
               OptimisticValueInitializer<OptimizationCriterion::EXPC>,
               OptimisticValueReporter<OptimizationCriterion::EXPC>,
               Factory > (parser);
#endif
    default:
        break;
    }
    std::cerr << "Unsupported optimization criterion." << std::endl
              << "Make sure that the constants are set correctly in " << std::endl
              << "  prob_search_engine_factory.h" << std::endl
              << "  tarjan_dfs_factory.h" << std::endl
              << "and recompile it." << std::endl;
    return NULL;
}

template<template<typename> class Factory>
static SearchEngine *parse_dual(OptionParser &parser)
{
    switch (g_optimization_criterion) {
    case OptimizationCriterion::MAXP:
        return generate_dual <
               ProbSearchSpaceDualMax,
               BiValueInitializer<OptimizationCriterion::MAXP>,
               LRTDPBiValueReporter<OptimizationCriterion::MAXP>,
               Factory > (parser);
#if COMPILE_OPTIMIZATION_CRITERION_EXPC
    case OptimizationCriterion::EXPC:
        return generate_dual <
               ProbSearchSpaceDualMin,
               BiValueInitializer<OptimizationCriterion::EXPC>,
               LRTDPBiValueReporter<OptimizationCriterion::EXPC>,
               Factory > (parser);
#endif
    default:
        break;
    }
    std::cerr << "Unsupported optimization criterion." << std::endl
              << "Make sure that the constants are set correctly in " << std::endl
              << "  prob_search_engine_factory.h" << std::endl
              << "  tarjan_dfs_factory.h" << std::endl
              << "and recompile it." << std::endl;
    return NULL;
}

template<template<typename> class Factory>
static SearchEngine *parse(OptionParser &parser)
{
    add_options_to_parser(parser);
    if (g_use_upper_and_lower) {
        parse_dual<Factory>(parser);
    } else {
        parse_single<Factory>(parser);
    }
    return NULL;
}
/*
template<template<typename> class Factory>
static SearchEngine *parse(OptionParser &parser)
{
    tarjan_dfs_factory::add_options_to_parser(parser);
    if (g_use_upper_and_lower) {
        switch (g_optimization_criterion) {
        case OptimizationCriterion::MAXP:
            return tarjan_dfs_factory::generate_dual <
                   ProbSearchSpaceDualMax,
                   BiValueInitializer<OptimizationCriterion::MAXP>,
                   LRTDPBiValueReporter<OptimizationCriterion::MAXP>,
                   Factory > (parser);
        case OptimizationCriterion::EXPC:
            return tarjan_dfs_factory::generate_dual <
                   ProbSearchSpaceDualMin,
                   BiValueInitializer<OptimizationCriterion::EXPC>,
                   LRTDPBiValueReporter<OptimizationCriterion::EXPC>,
                   Factory > (parser);
        }
    } else {
        switch (g_optimization_criterion) {
        case OptimizationCriterion::MAXP:
            return tarjan_dfs_factory::generate_upper <
                   ProbSearchSpaceMax,
                   OptimisticValueInitializer<OptimizationCriterion::MAXP>,
                   OptimisticValueReporter<OptimizationCriterion::MAXP>,
                   Factory > (parser);
        case OptimizationCriterion::EXPC:
            return tarjan_dfs_factory::generate_upper <
                   ProbSearchSpaceMin,
                   OptimisticValueInitializer<OptimizationCriterion::EXPC>,
                   OptimisticValueReporter<OptimizationCriterion::EXPC>,
                   Factory > (parser);
        }
    }
    return NULL;
}
*/

}

#endif

