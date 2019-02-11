
#include "acyclic_dfs_factory.h"

#if COMPILE_ACYCLIC_DFS

#include "acyclic_dfs.h"

#include "globals.h"

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

template<template<typename> class StateInfoWrapper, typename SStateInfo, template<template<typename> class, typename> class StateWrapper, template<typename> class SState, template<template<typename, typename> class, typename , template<typename> class> class SSearchSpace, typename ValueInitializer, typename ValueReporter, template<typename> class Callback, typename TieBreaking, typename CheckSolved, typename VLookup>
struct AcyclicDFSMaxP {
    struct StateInfo : StateInfoWrapper<LargelinkStateInfo<SStateInfo> > {
        StateInfo(int sid, int budget) :
            StateInfoWrapper<LargelinkStateInfo<SStateInfo> >(sid, budget) {}
    };
    template<typename Info> struct _State : public LargelinkState<SState, Info> {
        _State(size_t a, Info &b, unsigned u) : LargelinkState<SState, Info>(a, b, u) {}
    };
    template<typename Info> struct State : public StateWrapper<_State, Info> {
        State(size_t a, Info &b, unsigned u) : StateWrapper<_State, Info>(a, b, u) {}
    };
    template<template<typename, typename> class A, typename B, template<typename> class C>
    struct SearchSpace : public SSearchSpace<A, B, C> {
        SearchSpace(A<C<B>, typename SSearchSpace<A, B, C>::StateIDLookup>
                    &state_space) : SSearchSpace<A, B, C>(state_space) {}
    };

    template<template<typename, typename> class StateSpace>
    SearchEngine *create(Options &opts) const {
        opts.set<ValueInitializer *>("initializer",
                                     new ValueInitializer());
        opts.set<ValueReporter *>("reporter", new ValueReporter());

        return new
               AcyclicDFSEngine<StateInfo, State, SearchSpace, StateSpace, ValueInitializer, ValueReporter, Callback, TieBreaking, CheckSolved, VLookup>
               (opts);
    }
};

static void add_options_to_parser(OptionParser &parser)
{
    prob_search_engine::add_options_to_parser(parser);

    std::vector<std::string> tiebreaking;
    action_tie_breaking::get_options(tiebreaking);
    parser.add_enum_option("tiebreaking", tiebreaking, "", tiebreaking.front());

    parser.add_option<float>("epsilon", "", "0");

    parser.add_option<bool>("labeling", "", "false");
    parser.add_option<bool>("fw_updates", "", "true");
    parser.add_option<bool>("bw_updates", "", "false");
    parser.add_option<bool>("inconsistent", "", "false");
    parser.add_option<bool>("terminate_trial", "", "false");

    parser.add_option<bool>("vi", "", "true");

    parser.add_option<bool>("cache_flags", "", "false");
}

static void consistency_check(const Options &opts)
{
    if (!opts.get<bool>("vi") && !opts.get<bool>("labeling")) {
        std::cerr << "you have to set either vi or labeling" << std::endl;
        exit_with(EXIT_CRITICAL_ERROR);
    }
    if (!opts.get<bool>("bw_updates") && !opts.get<bool>("vi")) {
        std::cerr << "you have to set at least one of vi and bw_update" << std::endl;
        exit_with(EXIT_CRITICAL_ERROR);
    }
    if (opts.get<bool>("inconsistent") && !opts.get<bool>("fw_updates")) {
        std::cerr << "inconsistencies can be only checked by doing forward updates" <<
                  std::endl;
        exit_with(EXIT_CRITICAL_ERROR);
    }
    if (opts.get<bool>("labeling") && !opts.get<bool>("fw_updates")) {
        std::cerr <<
                  "in order to label states solved, we have to check for inconsistent states which requires forward updates"
                  << std::endl;
        exit_with(EXIT_CRITICAL_ERROR);
    }
}

template<template<template<typename, typename> class, typename, template<typename> class> class t_search_space, typename t_vinit, typename t_vreport, template<typename> class Factory>
static SearchEngine *generate_upper(OptionParser &parser)
{
    Options opts = parser.parse();
    if (!parser.dry_run()) {
        consistency_check(opts);
        if (opts.get<bool>("vi")) {
            switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
            case action_tie_breaking::ARBITRARY:
                return Factory < AcyclicDFSMaxP <
                       SingleVStateInfo,
                       BasicStateInfo,
                       SingleVState,
                       BasicState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       TieBreakingArbitrary,
                       VIConverged,
                       OptimisticV > >::create(parser);
#if COMPILE_TIEBREAKING_MINEXPH
            case action_tie_breaking::MINEXPH:
                return Factory < AcyclicDFSMaxP <
                       SingleVStateInfo,
                       HStateInfo,
                       SingleVState,
                       HState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       TieBreakingMinExpH,
                       VIConverged,
                       OptimisticV > >::create(parser);
#endif
#if COMPILE_TIEBREAKING_MINEXPGAP
            case action_tie_breaking::MINEXPGAP:
                std::cerr << "upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_MAXEXPGAP
            case action_tie_breaking::MAXEXGAP:
                std::cerr << "upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < AcyclicDFSMaxP <
                       SingleVStateInfo,
                       PreferredStateInfo,
                       SingleVState,
                       PreferredState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       TieBreakingPreferredArbitrary,
                       VIConverged,
                       OptimisticV > >::create(parser);
#endif
            default:
                break;
            }
        } else {
            switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
            case action_tie_breaking::ARBITRARY:
                return Factory < AcyclicDFSMaxP <
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
                return Factory < AcyclicDFSMaxP <
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
#if COMPILE_TIEBREAKING_MAXEXPGAP
            case action_tie_breaking::MAXEXGAP:
                std::cerr << "upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < AcyclicDFSMaxP <
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
              << "  acyclic_dfs_factory.h" << std::endl
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
                return Factory < AcyclicDFSMaxP <
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       TieBreakingArbitrary,
                       VIConverged,
                       BiOptimisticV > >::create(parser);
#if COMPILE_TIEBREAKING_MINEXPH
            case action_tie_breaking::MINEXPH:
                return Factory < AcyclicDFSMaxP <
                       UpperAndLowerVStateInfo,
                       HStateInfo,
                       UpperAndLowerVState,
                       HState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       TieBreakingMinExpH,
                       VIConverged,
                       BiOptimisticV > >::create(parser);
#endif
#if COMPILE_TIEBREAKING_MINEXPGAP
            case action_tie_breaking::MINEXPGAP:
                return Factory < AcyclicDFSMaxP <
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       TieBreakingMinExpGap,
                       VIConverged,
                       BiOptimisticV > >::create(parser);
#endif
#if COMPILE_TIEBREAKING_MAXEXPGAP
            case action_tie_breaking::MAXEXGAP:
                std::cerr << "upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < AcyclicDFSMaxP <
                       UpperAndLowerVStateInfo,
                       PreferredStateInfo,
                       UpperAndLowerVState,
                       PreferredState,
                       t_search_space,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       TieBreakingPreferredArbitrary,
                       VIConverged,
                       BiOptimisticV > >::create(parser);
#endif
            default:
                break;
            }
        } else {
            switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
            case action_tie_breaking::ARBITRARY:
                return Factory < AcyclicDFSMaxP <
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
                return Factory < AcyclicDFSMaxP <
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
                return Factory < AcyclicDFSMaxP <
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
#if COMPILE_TIEBREAKING_MAXEXPGAP
            case action_tie_breaking::MAXEXGAP:
                std::cerr << "upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < AcyclicDFSMaxP <
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
              << "  acyclic_dfs_factory.h" << std::endl
              << "and recompile it." << std::endl;
    return NULL;
}

namespace prob_search_engine_factory
{
template<template<typename> class Factory>
static SearchEngine *parse_ahdfs(OptionParser &parser)
{
    add_options_to_parser(parser);
    if (g_use_upper_and_lower) {
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
    } else {
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
    }
    std::cerr << "Unsupported optimization criterion." << std::endl
              << "Make sure that the constants are set correctly in " << std::endl
              << "  prob_search_engine_factory.h" << std::endl
              << "  acyclic_dfs_factory.h" << std::endl
              << "and recompile it." << std::endl;
    return NULL;
}


Plugin<SearchEngine> ahdfs("ahdfs", parse_ahdfs<ProbabilisticSearchEngine>);
}

#endif

