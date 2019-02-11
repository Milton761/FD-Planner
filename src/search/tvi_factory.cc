
#include "tvi_factory.h"
#include "topological_vi.h"

#include "optimization_criteria.h"
#include "option_parser.h"

#include "action_tie_breaking.h"

template<template<template<typename, typename> class A, typename B, template<typename> class C> class SP, typename ValueInitializer, typename ValueReporter>
struct TVIFactory {
    typedef TVIStateInfo StateInfo;
    template<typename Info> struct State : public TVIState<Info> {
        State(size_t a, Info &b, unsigned u) : TVIState<Info>(a, b, u) {}
    };
    template<template<typename, typename> class A, typename B, template<typename> class C>
    struct SearchSpace : public SP<A, B, C> {
        SearchSpace(A<C<B>, typename SP<A, B, C>::StateIDLookup>
                    &state_space) : SP<A, B, C>(state_space) {}
    };

    template<template<typename, typename> class StateSpace>
    SearchEngine *create(Options &opts) const {
        opts.set<ValueInitializer*>("initializer",
                new ValueInitializer());
        opts.set<ValueReporter*>("reporter",
                new ValueReporter());
        return new
               TopologicalValueIteration<StateInfo, State, SearchSpace, StateSpace, ValueInitializer, ValueReporter, TieBreakingArbitrary>
               (opts);
    }
};

static SearchEngine *parse_tvi(OptionParser &parser)
{
    switch (g_optimization_criterion) {
    case OptimizationCriterion::MAXP:
        return prob_search_engine_factory::ProbabilisticSearchEngine<TVIFactory<
            ProbSearchSpaceMax,
            PessimisticValueInitializer<OptimizationCriterion::MAXP>,
            PessimisticValueReporter<OptimizationCriterion::MAXP> > >::create(parser);
#if COMPILE_OPTIMIZATION_CRITERION_EXPC
    case OptimizationCriterion::EXPC:
        return prob_search_engine_factory::ProbabilisticSearchEngine<TVIFactory<
            ProbSearchSpaceMin,
            PessimisticValueInitializer<OptimizationCriterion::EXPC>,
            PessimisticValueReporter<OptimizationCriterion::EXPC> > >::create(parser);
#endif
    default:
        break;
    }
    std::cerr << "Unsupported optimization criterion." << std::endl
              << "Make sure that the constants are set correctly in " << std::endl
              << "  prob_search_engine_factory.h" << std::endl
              << "  vi_factory.h" << std::endl
              << "and recompile it." << std::endl;
    return NULL;
}

Plugin<SearchEngine> prob_search_engine_factory::tvi("tvi", parse_tvi);

