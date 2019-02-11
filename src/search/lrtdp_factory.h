
#ifndef LRTDP_FACTORY_H
#define LRTDP_FACTORY_H

#include "prob_search_engine_factory.h"

#include "lrtdp.h"

#include "globals.h"

#include "optimization_criteria.h"
#include "action_tie_breaking.h"

#include "option_parser.h"
#include "plugin.h"

#include "utilities.h"

#include <iostream>
#include <vector>
#include <string>

#define COMPILE_LRTDP_BIAS_H 0
#define COMPILE_LRTDP_BIAS_GAP 0

namespace lrtdp_factory
{
/*
    To support the integration into FRET, the LRTDP factory additionally has to
    define the types
        typedef Vinit
        typedef Vreport
        typedef TieBreaking
    and addionally has to provide the function
        template<typename t_info, template<typename> class t_state, template<template<typename, typename> class, typename, template<typename> class> class t_search_space, template<typename, typename> class t_state_space, typename t_init, typename t_report, typename t_tie>
        SearchEngine *create(Options &opts)
*/
template<typename OSelector, typename Value, class ValueInitializer, typename ValueReporter, template<typename> class Callback, typename TTieBreaking, template<typename> class SStateInfoWrapper, typename SStateInfo, template<template<typename> class, typename> class SStateWrapper, template<typename> class SState, template<template<typename, typename> class, typename , template<typename> class> class SSearchSpace>
struct LRTDPFactory {
    typedef SStateInfoWrapper<SStateInfo> StateInfo;

    template<typename Info> struct State : public SStateWrapper<SState, Info> {
        State(size_t a, Info &b, unsigned u) : SStateWrapper<SState, Info>(a, b, u) {}
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
    SearchEngine *create(Options &opts) {
        return this->create<StateInfo, State, SearchSpace, StateSpace, Vinit, Vreport, TieBreaking>
               (opts);
    }

    template<typename t_info, template<typename> class t_state, template<template<typename, typename> class, typename, template<typename> class> class t_search_space, template<typename, typename> class t_state_space, typename t_init, typename t_report, typename t_tie>
    SearchEngine *create(Options &opts) {
        std::cout << "LRTDP-SAMPLER: seed = " << opts.get<int>("seed")
                  << ", epsilon_consistent = " << opts.get<bool>("epsconsist")
                  << ", epsilon = ";
        printf("%.8f\n", opts.get<float>("epsilon"));
        opts.set<OSelector *>("sampler",
                              new OSelector(opts.get<int>("seed"), opts.get<bool>("epsconsist"),
                                            opts.get<float>("epsilon")));
        opts.set<t_init *>("initializer", new t_init());
        opts.set<t_report *>("reporter", new t_report());
        return new
               LRTDP<t_info, t_state, t_search_space, t_state_space, t_init, t_report, Value, OSelector, Callback, t_tie, false>
               (opts);
    }
};

void add_options_to_parser(OptionParser &parser);
void copy_options_to_parser(const Options &opts, OptionParser &parser);

enum OutcomeSelection {
    DEFAULT = 0,
    GAP = 1,
    H = 2,
};

template<template<template<typename, typename> class, typename, template<typename> class> class t_search_space, typename t_vinit, typename t_vreport, template<typename> class Factory>
SearchEngine *generate_upper(OptionParser &parser)
{
    Options opts = parser.parse();
    if (!parser.dry_run()) {
        switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
        case action_tie_breaking::ARBITRARY: {
            switch (OutcomeSelection(opts.get_enum("oselect"))) {
            case DEFAULT:
                return Factory < LRTDPFactory <
                       LRTDPStdSampler,
                       OptimisticV,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       TieBreakingArbitrary,
                       SingleVStateInfo,
                       BasicStateInfo,
                       SingleVState,
                       BasicState,
                       t_search_space > >::create(parser);
#if COMPILE_LRTDP_BIAS_H
            case H:
                return Factory < LRTDPFactory <
                       BiasedLRTDPSampler<BiasedH>,
                       OptimisticV,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       TieBreakingArbitrary,
                       SingleVStateInfo,
                       HStateInfo,
                       SingleVState,
                       HState,
                       t_search_space > >::create(parser);
#endif
#if COMPILE_LRTDP_BIAS_GAP
            case GAP:
                std::cerr <<
                          "LRTDP with just upper bound does not support gap based biasing" <<
                          std::endl;
                return NULL;
#endif
            default:
                break;
            }
        }

#if COMPILE_TIEBREAKING_PREFERRED
        case action_tie_breaking::PREFERRED: {
            switch (OutcomeSelection(opts.get_enum("oselect"))) {
            case DEFAULT:
                return Factory < LRTDPFactory <
                       LRTDPStdSampler,
                       OptimisticV,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       TieBreakingPreferredArbitrary,
                       SingleVStateInfo,
                       PreferredStateInfo,
                       SingleVState,
                       PreferredState,
                       t_search_space > >::create(parser);
#if COMPILE_LRTDP_BIAS_H
            case H:
                return Factory < LRTDPFactory <
                       BiasedLRTDPSampler<BiasedH>,
                       OptimisticV,
                       t_vinit,
                       t_vreport,
                       StorePrefH,
                       TieBreakingPreferredArbitrary,
                       SingleVStateInfo,
                       PreferredHStateInfo,
                       SingleVState,
                       PreferredHState,
                       t_search_space > >::create(parser);
#endif
#if COMPILE_LRTDP_BIAS_GAP
            case GAP:
                std::cerr <<
                          "LRTDP with just upper bound does not support gap based biasing" <<
                          std::endl;
                return NULL;
#endif
            default:
                break;
            }
        }
#endif

#if COMPILE_TIEBREAKING_MINEXPH
        case action_tie_breaking::MINEXPH: {
            switch (OutcomeSelection(opts.get_enum("oselect"))) {
            case DEFAULT:
                return Factory < LRTDPFactory <
                       LRTDPStdSampler,
                       OptimisticV,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       TieBreakingMinExpH,
                       SingleVStateInfo,
                       HStateInfo,
                       SingleVState,
                       HState,
                       t_search_space > >::create(parser);
#if COMPILE_LRTDP_BIAS_H
            case H:
                return Factory < LRTDPFactory <
                       BiasedLRTDPSampler<BiasedH>,
                       OptimisticV,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       TieBreakingMinExpH,
                       SingleVStateInfo,
                       HStateInfo,
                       SingleVState,
                       HState,
                       t_search_space > >::create(parser);
#endif
#if COMPILE_LRTDP_BIAS_GAP
            case GAP:
                std::cerr <<
                          "LRTDP with just upper bound does not support gap based biasing" <<
                          std::endl;
                return NULL;
#endif
            default:
                break;
            }
        }
#endif
        default:
            break;
        }
    }
    std::cerr << "Unsupported LRTDP parameter encountered." << std::endl
              << "Make sure that the constants are set correctly in " << std::endl
              << "  prob_search_engine_factory.h" << std::endl
              << "  lrtdp_factory.h" << std::endl
              << "and recompile it." << std::endl;
    return NULL;
}

template<template<template<typename, typename> class, typename, template<typename> class> class t_search_space, typename t_vinit, typename t_vreport, template<typename> class Factory>
SearchEngine *generate_dual(OptionParser &parser)
{
    Options opts = parser.parse();
    if (!parser.dry_run()) {
        switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
        case action_tie_breaking::ARBITRARY: {
            switch (OutcomeSelection(opts.get_enum("oselect"))) {
            case DEFAULT:
                return Factory < LRTDPFactory <
                       LRTDPStdSampler,
                       BiOptimisticV,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       TieBreakingArbitrary,
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space > >::create(parser);
#if COMPILE_LRTDP_BIAS_H
            case H:
                return Factory < LRTDPFactory <
                       BiasedLRTDPSampler<BiasedH>,
                       BiOptimisticV,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       TieBreakingArbitrary,
                       UpperAndLowerVStateInfo,
                       HStateInfo,
                       UpperAndLowerVState,
                       HState,
                       t_search_space > >::create(parser);
#endif
#if COMPILE_LRTDP_BIAS_GAP
            case GAP:
                return Factory < LRTDPFactory <
                       BiasedLRTDPSampler<BiasedGap>,
                       BiOptimisticV,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       TieBreakingArbitrary,
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space > >::create(parser);
#endif
            default:
                break;
            }
        }

#if COMPILE_TIEBREAKING_PREFERRED
        case action_tie_breaking::PREFERRED: {
            switch (OutcomeSelection(opts.get_enum("oselect"))) {
            case DEFAULT:
                return Factory < LRTDPFactory <
                       LRTDPStdSampler,
                       BiOptimisticV,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       TieBreakingPreferredArbitrary,
                       UpperAndLowerVStateInfo,
                       PreferredStateInfo,
                       UpperAndLowerVState,
                       PreferredState,
                       t_search_space > >::create(parser);
#if COMPILE_LRTDP_BIAS_H
            case H:
                return Factory < LRTDPFactory <
                       BiasedLRTDPSampler<BiasedH>,
                       BiOptimisticV,
                       t_vinit,
                       t_vreport,
                       StorePrefH,
                       TieBreakingPreferredArbitrary,
                       UpperAndLowerVStateInfo,
                       PreferredHStateInfo,
                       UpperAndLowerVState,
                       PreferredHState,
                       t_search_space > >::create(parser);
#endif
#if COMPILE_LRTDP_BIAS_GAP
            case GAP:
                return Factory < LRTDPFactory <
                       BiasedLRTDPSampler<BiasedGap>,
                       BiOptimisticV,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       TieBreakingPreferredArbitrary,
                       UpperAndLowerVStateInfo,
                       PreferredStateInfo,
                       UpperAndLowerVState,
                       PreferredState,
                       t_search_space > >::create(parser);
#endif
            default:
                break;
            }
        }
#endif

#if COMPILE_TIEBREAKING_MINEXPH
        case action_tie_breaking::MINEXPH: {
            switch (OutcomeSelection(opts.get_enum("oselect"))) {
            case DEFAULT:
                return Factory < LRTDPFactory <
                       LRTDPStdSampler,
                       BiOptimisticV,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       TieBreakingMinExpH,
                       UpperAndLowerVStateInfo,
                       HStateInfo,
                       UpperAndLowerVState,
                       HState,
                       t_search_space > >::create(parser);
#if COMPILE_LRTDP_BIAS_H
            case H:
                return Factory < LRTDPFactory <
                       BiasedLRTDPSampler<BiasedH>,
                       BiOptimisticV,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       TieBreakingMinExpH,
                       UpperAndLowerVStateInfo,
                       HStateInfo,
                       UpperAndLowerVState,
                       HState,
                       t_search_space > >::create(parser);
#endif
#if COMPILE_LRTDP_BIAS_GAP
            case GAP:
                return Factory < LRTDPFactory <
                       BiasedLRTDPSampler<BiasedGap>,
                       BiOptimisticV,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       TieBreakingMinExpH,
                       UpperAndLowerVStateInfo,
                       HStateInfo,
                       UpperAndLowerVState,
                       HState,
                       t_search_space > >::create(parser);
#endif
            default:
                break;
            }
        }
#endif

#if COMPILE_TIEBREAKING_MINEXPGAP
        case action_tie_breaking::MINEXPGAP: {
            switch (OutcomeSelection(opts.get_enum("oselect"))) {
            case DEFAULT:
                return Factory < LRTDPFactory <
                       LRTDPStdSampler,
                       BiOptimisticV,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       TieBreakingMinExpGap,
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space > >::create(parser);
#if COMPILE_LRTDP_BIAS_H
            case H:
                return Factory < LRTDPFactory <
                       BiasedLRTDPSampler<BiasedH>,
                       BiOptimisticV,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       TieBreakingMinExpGap,
                       UpperAndLowerVStateInfo,
                       HStateInfo,
                       UpperAndLowerVState,
                       HState,
                       t_search_space > >::create(parser);
#endif
#if COMPILE_LRTDP_BIAS_GAP
            case GAP:
                return Factory < LRTDPFactory <
                       BiasedLRTDPSampler<BiasedGap>,
                       BiOptimisticV,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       TieBreakingMinExpGap,
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space > >::create(parser);
#endif
            default:
                break;
            }
        }
#endif

#if COMPILE_TIEBREAKING_MAXEXPGAP
        case action_tie_breaking::MAXEXGAP: {
            switch (OutcomeSelection(opts.get_enum("oselect"))) {
            case DEFAULT:
                return Factory < LRTDPFactory <
                       LRTDPStdSampler,
                       BiOptimisticV,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       TieBreakingMaxExpGap,
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space > >::create(parser);
#if COMPILE_LRTDP_BIAS_H
            case H:
                return Factory < LRTDPFactory <
                       BiasedLRTDPSampler<BiasedH>,
                       BiOptimisticV,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       TieBreakingMaxExpGap,
                       UpperAndLowerVStateInfo,
                       HStateInfo,
                       UpperAndLowerVState,
                       HState,
                       t_search_space > >::create(parser);
#endif
#if COMPILE_LRTDP_BIAS_GAP
            case GAP:
                return Factory < LRTDPFactory <
                       BiasedLRTDPSampler<BiasedGap>,
                       BiOptimisticV,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       TieBreakingMaxExpGap,
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space > >::create(parser);
#endif
            default:
                break;
            }
        }
#endif
        default:
            break;
        }
    }
    std::cerr << "Unsupported LRTDP parameter encountered." << std::endl
              << "Make sure that the constants are set correctly in " << std::endl
              << "  prob_search_engine_factory.h" << std::endl
              << "  lrtdp_factory.h" << std::endl
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
              << "  lrtdp_factory.h" << std::endl
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
              << "  lrtdp_factory.h" << std::endl
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

}

namespace prob_search_engine_factory
{
extern Plugin<SearchEngine> lrtdp;
}


#endif
