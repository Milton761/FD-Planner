
#include "vi_factory.h"
#include "acyclic_value_iteration.h"
#include "action_tie_breaking.h"

/* Factory class to generate an instance of the VI implementation for
   acyclic state space */
/* Each factory has to define the following types:
     typedef StateInfo
     template<typename Info> struct State
     template<template<typename, typename> class StateSpace, typename StateInfo, template<typename> class State> struct SearchSpace
*/
/* Each factory has to define the function
     template<template<typename, typename> class StateSpace>
     SearchEngine *create(Options &opts)
*/
template<template<template<typename, typename> class A, typename B, template<typename> class C> class SP, typename ValueInitializer, typename ValueReporter>
struct AVIFactory {
    typedef AcyclicProbSearchNodeInfo StateInfo;
    template<typename Info> struct State : public AcyclicProbSearchNode<Info> {
        State(size_t a, Info &b, unsigned u) : AcyclicProbSearchNode<Info>(a, b, u) {}
    };
    template<template<typename, typename> class A, typename B, template<typename> class C>
    struct SearchSpace : public SP<A, B, C> {
        SearchSpace(typename SP<A, B, C>::t_state_space &state_space)
            : SP<A, B, C>(state_space) {}
    };
    template<template<typename,typename> class StateSpace>
    SearchEngine *create(Options &opts) const {
        opts.set<ValueInitializer*>("initializer", new ValueInitializer());
        opts.set<ValueReporter*>("reporter", new ValueReporter());
        return new AcyclicValueIteration<StateInfo, State, SearchSpace, StateSpace, ValueInitializer, ValueReporter, TieBreakingArbitrary>(opts);
    }
};

// Wrapper to handles the various global options.
// The only globals option relevant for VI is the optimization criterion.
// Note that both SearchSpace and ValueInitializer / ValueReporter always depend
// on the optimization criterion.
static SearchEngine *parse_avi(OptionParser &parser)
{
    switch (g_optimization_criterion) {
    case OptimizationCriterion::MAXP:
        return prob_search_engine_factory::ProbabilisticSearchEngine<AVIFactory<
            ProbSearchSpaceMax,
            PessimisticValueInitializer<OptimizationCriterion::MAXP>,
            PessimisticValueReporter<OptimizationCriterion::MAXP> > >::create(parser);
#if COMPILE_OPTIMIZATION_CRITERION_EXPC
    case OptimizationCriterion::EXPC:
        return prob_search_engine_factory::ProbabilisticSearchEngine<AVIFactory<
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

Plugin<SearchEngine> prob_search_engine_factory::avi("avi", parse_avi);

