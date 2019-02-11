
#include "aol_factory.h"
#include "greedy_ao_search.h"

#include "optimization_criteria.h"
#include "action_tie_breaking.h"

#include "option_parser.h"
#include "plugin.h"

#include "utilities.h"

#include <iostream>
#include <vector>
#include <string>

template<typename ValueInitializer, typename ValueReporter, typename ParentSelection, typename UpdateHandler, typename TieBreaking, typename SStateInfo, template<typename> class SState, template<template<typename, typename> class, typename , template<typename> class> class SSearchSpace>
struct AOLSearchFactory {
    typedef PreferredOpStateInfo<SStateInfo> StateInfo;
    template<typename Info> struct State : public PreferredOpState<SState, Info> {
        State(size_t a, Info &b, unsigned u) : PreferredOpState<SState, Info>(a, b, u) {}
    };
    template<template<typename, typename> class A, typename B, template<typename> class C>
    struct SearchSpace : public SSearchSpace<A, B, C> {
        SearchSpace(A<C<B>, typename SSearchSpace<A, B, C>::StateIDLookup>
                    &state_space) : SSearchSpace<A, B, C>(state_space) {}
    };
    template <typename State, typename SearchSpace, typename StateSpace, typename TTieB>
    struct AllParentsValueUpdate : public
            OrderedValueUpdater<State, SearchSpace, StateSpace, ParentSelection, UpdateHandler, TTieB> {
        AllParentsValueUpdate(SearchSpace &a, StateSpace &b, float eps)
            : OrderedValueUpdater<State, SearchSpace, StateSpace, ParentSelection, UpdateHandler, TTieB>
            (a, b, eps) {}
    };

    template<template<typename, typename> class StateSpace>
    SearchEngine *create(Options &opts) const {
        opts.set<AOValueInitializer<ValueInitializer>*>("initializer",
                new AOValueInitializer<ValueInitializer>());
        opts.set<ValueReporter *>("reporter", new ValueReporter());

        if (!opts.contains("preferred")) {
            opts.set<Heuristic *>("preferred", NULL);
        }
        return _gen1<
            StateSpace,
            PreferredOpenListAdder,
            TieBreakingPreferred<TieBreaking>>(opts);
    }

    template<template<typename, typename> class StateSpace, template<typename, template<typename> class> class TipList, template<typename> class Callback, typename TIES>
    struct _gen2 {
        template<typename A, typename B, typename C>
        struct OOSelector : public
                AOLOutcomeSelector<A, B, C, TipList<AOLSearchFactory::StateInfo, AOLSearchFactory::State> > {
            OOSelector(TipList<AOLSearchFactory::StateInfo, AOLSearchFactory::State> *l,
                       TipList<AOLSearchFactory::StateInfo, AOLSearchFactory::State> *pl) :
                AOLOutcomeSelector<A, B, C, TipList<AOLSearchFactory::StateInfo, AOLSearchFactory::State> >
                (l, pl) {}
        };
        SearchEngine *operator()(const Options &opts0, Options &opts) {
            using List = TipList<AOLSearchFactory::StateInfo, AOLSearchFactory::State>;
            using AO = AOSearch<
                StateInfo,
                State,
                SearchSpace,
                StateSpace,
                AOValueInitializer<ValueInitializer>,
                ValueReporter,
                AllParentsValueUpdate,
                _gen2::OOSelector,
                OnNewStateStoreHAndPreferredOps,
                Callback<List>,
                TIES,
                false>;

            List *l = new List(opts0);
            List *pl = new List(opts0);

            opts.set<OOSelector<typename AO::Node, typename AO::SearchSpace, typename AO::StateSpace>*>("oselector",
                    new OOSelector<typename AO::Node, typename AO::SearchSpace, typename AO::StateSpace>
                    (l, pl));
            opts.set<OnNewStateStoreHAndPreferredOps<typename AO::Node> *>("callback",
                                                new OnNewStateStoreHAndPreferredOps<typename AO::Node>(opts.get<Heuristic *>("preferred")));
            opts.set<Callback<List> *>("callbackre", new Callback<List>(l, pl));

            return new AO(opts);
        }
    };

    template<template<typename, typename> class StateSpace, template<typename> class Callback, typename TIES>
    SearchEngine *_gen1(Options &opts) const {
        ao_tip_list::TipListFactory *f =
            opts.get<ao_tip_list::TipListFactory *>("tiplist");

        switch (f->get_type()) {
#if COMPILE_AOL_TIPLIST_FIFO
        case ao_tip_list::FIFO: {
            return AOLSearchFactory::_gen2<StateSpace, ao_tip_list::FIFOTipList, Callback, TIES>()
                   (f->get_options(), opts);
        }
#endif
#if COMPILE_AOL_TIPLIST_LIFO
        case ao_tip_list::LIFO: {
            return AOLSearchFactory::_gen2<StateSpace, ao_tip_list::LIFOTipList, Callback, TIES>()
                   (f->get_options(), opts);
        }
#endif
#if COMPILE_AOL_TIPLIST_DASC
        case ao_tip_list::DISTANCE_ASC: {
            return AOLSearchFactory::_gen2<StateSpace, ao_tip_list::DistanceTipList, Callback, TIES>()
                   (f->get_options(), opts);
        }
#endif
#if COMPILE_AOL_TIPLIST_DDESC
        case ao_tip_list::DISTANCE_DESC: {
            return AOLSearchFactory::_gen2<StateSpace, ao_tip_list::DistanceDescTipList, Callback, TIES>()
                   (f->get_options(), opts);
        }
#endif
#if COMPILE_AOL_TIPLIST_HASC
        case ao_tip_list::HEURISTIC_ASC: {
            return AOLSearchFactory::_gen2<StateSpace, ao_tip_list::MinHeuristicTipList, Callback, TIES>()
                   (f->get_options(), opts);
        }
#endif
#if COMPILE_AOL_TIPLIST_HDESC
        case ao_tip_list::HEURISTIC_DESC: {
            return AOLSearchFactory::_gen2<StateSpace, ao_tip_list::MaxHeuristicTipList, Callback, TIES>()
                   (f->get_options(), opts);
        }
#endif
#if COMPILE_AOL_TIPLIST_DD
        case ao_tip_list::DD: {
            return AOLSearchFactory::_gen2<StateSpace, ao_tip_list::DDTipList, Callback, TIES>()
                   (f->get_options(), opts);
        }
#endif
#if COMPILE_AOL_TIPLIST_BASC
        case ao_tip_list::BUDGET_ASC: {
            return AOLSearchFactory::_gen2<StateSpace, ao_tip_list::BudgetAscTipList, Callback, TIES>()
                   (f->get_options(), opts);
#endif
#if COMPILE_AOL_TIPLIST_BDESC
        }
        case ao_tip_list::BUDGET_DESC: {
            return AOLSearchFactory::_gen2<StateSpace, ao_tip_list::BudgetDescTipList, Callback, TIES>()
                   (f->get_options(), opts);
        }
#endif
#if COMPILE_AOL_TIPLIST_DDd
        case ao_tip_list::DDd: {
            return AOLSearchFactory::_gen2<StateSpace, ao_tip_list::DDdTipList, Callback, TIES>()
                   (f->get_options(), opts);
        }
#endif
        default:
            std::cerr << "Unsupported AO*|L parameter encountered." << std::endl
                      << "Make sure that the constants are set correctly in " << std::endl
                      << "  prob_search_engine_factory.h" << std::endl
                      << "  aol_factory.h" << std::endl
                      << "and recompile it." << std::endl;
            exit_with(EXIT_CRITICAL_ERROR);
            break;
        }
    }
};

static void _add_options_to_parser(OptionParser &parser)
{
    prob_search_engine::add_options_to_parser(parser);
    parser.add_option<float>("epsilon", "", "0");
    parser.add_option<ao_tip_list::TipListFactory *>("tiplist", "", "lifo");
    parser.add_option<Heuristic *>("preferred", "", OptionParser::NONE);
    std::vector<std::string> tiebreaking;
    action_tie_breaking::get_options(tiebreaking);
    parser.add_enum_option("tiebreaking", tiebreaking, "", tiebreaking.front());
}


template<template<template<typename, typename> class, typename, template<typename> class> class t_search_space, typename t_vinit, typename t_vreport>
static SearchEngine *parse_wrapper(OptionParser &parser)
{
    Options opts = parser.parse();
    if (!parser.dry_run()) {
        switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
        case action_tie_breaking::ARBITRARY:
            return prob_search_engine_factory::_parse <
                   AOLSearchFactory<
                   t_vinit,
                   t_vreport,
                   AllParentsSelection,
                   NoUpdateHandler,
                   TieBreakingArbitrary,
                   DepthStateInfo<HStateInfo>,
                   DepthHState,
                   t_search_space> >(parser);
        default:
            std::cerr << "AO*|L only supports arbitrary tie breaking" << endl;
            return NULL;
        }
    }
    return NULL;
}

static SearchEngine *parse_aol(OptionParser &parser) {
    _add_options_to_parser(parser);
    switch (g_optimization_criterion) {
    case OptimizationCriterion::MAXP:
        return parse_wrapper<
            ProbSearchSpaceMax,
            PessimisticValueInitializer<OptimizationCriterion::MAXP>,
            PessimisticValueReporter<OptimizationCriterion::MAXP> >(parser);
#if COMPILE_OPTIMIZATION_CRITERION_EXPC
    case OptimizationCriterion::EXPC:
        return parse_wrapper<
            ProbSearchSpaceMin,
            PessimisticValueInitializer<OptimizationCriterion::EXPC>,
            PessimisticValueReporter<OptimizationCriterion::EXPC> >(parser);
#endif
    default:
        break;
    }
    std::cerr << "Unsupported optimization criterion." << std::endl
              << "Make sure that the constants are set correctly in " << std::endl
              << "  prob_search_engine_factory.h" << std::endl
              << "  aol_factory.h" << std::endl
              << "and recompile it." << std::endl;
    return NULL;
}

namespace prob_search_engine_factory {
    Plugin<SearchEngine> aol("aol", parse_aol);
}

