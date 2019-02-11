
#include "ao_factory.h"
#include "greedy_ao_search.h"

#include "globals.h"

#include "optimization_criteria.h"
#include "action_tie_breaking.h"

#include "option_parser.h"
#include "plugin.h"

#include "utilities.h"

#include <iostream>
#include <vector>
#include <string>

namespace ao_factory
{

template<template<typename, typename, typename> class OSelector, typename ValueInitializer, typename ValueReporter, template<typename> class Callback, typename ParentSelection, typename UpdateHandler, typename TieBreaking, template<typename> class SStateInfoWrapper, typename SStateInfo, template<template<typename> class, typename> class SStateWrapper, template<typename> class SState, template<template<typename, typename> class, typename , template<typename> class> class SSearchSpace>
struct AOSearchFactory {
    typedef SStateInfoWrapper<SStateInfo> StateInfo;
    template<typename Info> struct State : public SStateWrapper<SState, Info> {
        State(size_t a, Info &b, unsigned u) : SStateWrapper<SState, Info>(a, b, u) {}
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
    void add_oselector(Options &opts) const {
        using AO =
            AOSearch <
            StateInfo,
            State,
            SearchSpace,
            StateSpace,
            AOValueInitializer<ValueInitializer>,
            ValueReporter,
            AllParentsValueUpdate,
            OSelector,
            Callback,
            DoNothingOnSuccessor,
            TieBreaking,
            false >;

        opts.set<OSelector<typename AO::Node, typename AO::SearchSpace, typename AO::StateSpace>*>("oselector",
                new OSelector<typename AO::Node, typename AO::SearchSpace, typename AO::StateSpace>());
        opts.set<Callback<typename AO::Node>*>("callback",
                                               new Callback<typename AO::Node>());
    }

    template<template<typename, typename> class StateSpace>
    SearchEngine *create(Options &opts) const {
        opts.set<AOValueInitializer<ValueInitializer>*>("initializer",
                new AOValueInitializer<ValueInitializer>());
        opts.set<ValueReporter *>("reporter", new ValueReporter());
        opts.set<DoNothingOnSuccessor *>("callbackre", new DoNothingOnSuccessor());

        add_oselector<StateSpace>(opts);
        return new
               AOSearch<StateInfo, State, SearchSpace, StateSpace, AOValueInitializer<ValueInitializer>, ValueReporter, AllParentsValueUpdate, OSelector, Callback, DoNothingOnSuccessor, TieBreaking, false>
               (opts);
    }
};

static void add_options_to_parser(OptionParser &parser)
{
    prob_search_engine::add_options_to_parser(parser);
    std::vector<std::string> oselect;
    oselect.push_back("arbitrary");
    oselect.push_back("maxp");
    oselect.push_back("preferred");
    oselect.push_back("gap");
    oselect.push_back("minh");
    parser.add_enum_option("oselect", oselect, "", "arbitrary");

    std::vector<std::string> tiebreaking;
    action_tie_breaking::get_options(tiebreaking);
    parser.add_enum_option("tiebreaking", tiebreaking, "", tiebreaking.front());

    parser.add_option<float>("epsilon", "", "0");
}

enum OutcomeSelection {
    ARBITRARY = 0,
    MOST_LIKELY = 1,
    PREFERRED = 2,
    GAP = 3,
    MIN_H = 4,
};

template<template<template<typename, typename> class, typename, template<typename> class> class t_search_space, typename t_vinit, typename t_vreport, template<typename> class Factory, typename PaSelect>
static SearchEngine *generate_upper(OptionParser &parser)
{
    Options opts = parser.parse();
    if (!parser.dry_run()) {
        switch (OutcomeSelection(opts.get_enum("oselect"))) {
        case ARBITRARY: {
            switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
            case action_tie_breaking::ARBITRARY:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorArbitrary,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingArbitrary,
                       SingleVStateInfo,
                       BasicStateInfo,
                       SingleVState,
                       BasicState,
                       t_search_space > > ::create(parser);
#if COMPILE_TIEBREAKING_MINEXPH
            case action_tie_breaking::MINEXPH:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorArbitrary,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMinExpH,
                       SingleVStateInfo,
                       HStateInfo,
                       SingleVState,
                       HState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_MINEXPGAP
            case action_tie_breaking::MINEXPGAP:
                std::cerr << "AO* with just upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_MAXEXPGAP
            case action_tie_breaking::MAXEXGAP:
                std::cerr << "AO* with just upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorArbitrary,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingPreferredArbitrary,
                       SingleVStateInfo,
                       PreferredStateInfo,
                       SingleVState,
                       PreferredState,
                       t_search_space > > ::create(parser);
#endif
            default:
                break;

            }
        }

#if COMPILE_AO_OSELECT_MOST_LIKELY
        case MOST_LIKELY: {
            switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
            case action_tie_breaking::ARBITRARY:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorMostLikely,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingArbitrary,
                       SingleVStateInfo,
                       BasicStateInfo,
                       SingleVState,
                       BasicState,
                       t_search_space > > ::create(parser);
#if COMPILE_TIEBREAKING_MINEXPH
            case action_tie_breaking::MINEXPH:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorMostLikely,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMinExpH,
                       SingleVStateInfo,
                       HStateInfo,
                       SingleVState,
                       HState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_MINEXPGAP
            case action_tie_breaking::MINEXPGAP:
                std::cerr << "AO* with just upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_MAXEXPGAP
            case action_tie_breaking::MAXEXGAP:
                std::cerr << "AO* with just upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorMostLikely,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingPreferredArbitrary,
                       SingleVStateInfo,
                       PreferredStateInfo,
                       SingleVState,
                       PreferredState,
                       t_search_space > > ::create(parser);
#endif
            }
            default:
                break;
            }
#endif

#if COMPILE_AO_OSELECT_PREFERRED
        case PREFERRED: {
            switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
            case action_tie_breaking::ARBITRARY:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorPreferred,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingArbitrary,
                       SingleVStateInfo,
                       PreferredStateInfo,
                       SingleVState,
                       PreferredState,
                       t_search_space > > ::create(parser);
#if COMPILE_TIEBREAKING_MINEXPH
            case action_tie_breaking::MINEXPH:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorPreferred,
                       t_vinit,
                       t_vreport,
                       StorePrefH,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMinExpH,
                       SingleVStateInfo,
                       PreferredHStateInfo,
                       SingleVState,
                       PreferredHState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_MINEXPGAP
            case action_tie_breaking::MINEXPGAP:
                std::cerr << "AO* with just upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_MAXEXPGAP
            case action_tie_breaking::MAXEXGAP:
                std::cerr << "AO* with just upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorPreferred,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingPreferredArbitrary,
                       SingleVStateInfo,
                       PreferredStateInfo,
                       SingleVState,
                       PreferredState,
                       t_search_space > > ::create(parser);
#endif
            }
            default:
                break;
            }
#endif

#if COMPILE_AO_OSELECT_GAP
        case GAP: {
            std::cerr <<
                      "AO* with just upper bound cannot use outcome selection based on gap" <<
                      std::endl;
            return NULL;
        }
#endif

#if COMPILE_AO_OSELECT_MIN_H
        case MIN_H: {
            switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
            case action_tie_breaking::ARBITRARY:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorMinH,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingArbitrary,
                       SingleVStateInfo,
                       HStateInfo,
                       SingleVState,
                       HState,
                       t_search_space > > ::create(parser);
#if COMPILE_TIEBREAKING_MINEXPH
            case action_tie_breaking::MINEXPH:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorMinH,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMinExpH,
                       SingleVStateInfo,
                       HStateInfo,
                       SingleVState,
                       HState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_MINEXPGAP
            case action_tie_breaking::MINEXPGAP:
                std::cerr << "AO* with just upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_MAXEXPGAP
            case action_tie_breaking::MAXEXGAP:
                std::cerr << "AO* with just upper bound cannot use tiebreaking based on gap" <<
                          std::endl;
                exit_with(EXIT_CRITICAL_ERROR);
                return NULL;
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorMinH,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingArbitrary,
                       SingleVStateInfo,
                       PreferredHStateInfo,
                       SingleVState,
                       PreferredHState,
                       t_search_space > > ::create(parser);
            }
#endif

            default:
                break;
            }
#endif

        default:
            break;
        }
    }

    std::cerr << "Unsupported AO* parameter encountered." << std::endl
              << "Make sure that the constants are set correctly in " << std::endl
              << "  prob_search_engine_factory.h" << std::endl
              << "  ao_factory.h" << std::endl
              << "and recompile it." << std::endl;
    return NULL;
}


template<template<template<typename, typename> class, typename, template<typename> class> class t_search_space, typename t_vinit, typename t_vreport, template<typename> class Factory, typename PaSelect>
static SearchEngine *generate_dual(OptionParser &parser)
{
    Options opts = parser.parse();
    if (!parser.dry_run()) {
        switch (OutcomeSelection(opts.get_enum("oselect"))) {
        case ARBITRARY: {
            switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
            case action_tie_breaking::ARBITRARY:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorArbitrary,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingArbitrary,
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space > > ::create(parser);
#if COMPILE_TIEBREAKING_MINEXPH
            case action_tie_breaking::MINEXPH:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorArbitrary,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMinExpH,
                       UpperAndLowerVStateInfo,
                       HStateInfo,
                       UpperAndLowerVState,
                       HState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_MINEXPGAP
            case action_tie_breaking::MINEXPGAP:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorArbitrary,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMinExpGap,
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_MAXEXPGAP
            case action_tie_breaking::MAXEXGAP:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorArbitrary,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMaxExpGap,
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorArbitrary,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingPreferredArbitrary,
                       UpperAndLowerVStateInfo,
                       PreferredStateInfo,
                       UpperAndLowerVState,
                       PreferredState,
                       t_search_space > > ::create(parser);
#endif
            default:
                break;
            }
        }

#if COMPILE_AO_OSELECT_MOST_LIKELY
        case MOST_LIKELY: {
            switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
            case action_tie_breaking::ARBITRARY:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorMostLikely,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingArbitrary,
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space > > ::create(parser);
#if COMPILE_TIEBREAKING_MINEXPH
            case action_tie_breaking::MINEXPH:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorMostLikely,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMinExpH,
                       UpperAndLowerVStateInfo,
                       HStateInfo,
                       UpperAndLowerVState,
                       HState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_MINEXPGAP
            case action_tie_breaking::MINEXPGAP:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorMostLikely,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMinExpGap,
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_MAXEXPGAP
            case action_tie_breaking::MAXEXGAP:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorMostLikely,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMaxExpGap,
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorMostLikely,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingPreferredArbitrary,
                       UpperAndLowerVStateInfo,
                       PreferredStateInfo,
                       UpperAndLowerVState,
                       PreferredState,
                       t_search_space > > ::create(parser);
#endif
            default:
                break;
            }
        }
#endif

#if COMPILE_AO_OSELECT_PREFERRED
        case PREFERRED: {
            switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
            case action_tie_breaking::ARBITRARY:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorPreferred,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingArbitrary,
                       UpperAndLowerVStateInfo,
                       PreferredStateInfo,
                       UpperAndLowerVState,
                       PreferredState,
                       t_search_space > > ::create(parser);
#if COMPILE_TIEBREAKING_MINEXPH
            case action_tie_breaking::MINEXPH:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorPreferred,
                       t_vinit,
                       t_vreport,
                       StorePrefH,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMinExpH,
                       UpperAndLowerVStateInfo,
                       PreferredHStateInfo,
                       UpperAndLowerVState,
                       PreferredHState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_MINEXPGAP
            case action_tie_breaking::MINEXPGAP:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorPreferred,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMinExpGap,
                       UpperAndLowerVStateInfo,
                       PreferredStateInfo,
                       UpperAndLowerVState,
                       PreferredState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_MAXEXPGAP
            case action_tie_breaking::MAXEXGAP:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorPreferred,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMaxExpGap,
                       UpperAndLowerVStateInfo,
                       PreferredStateInfo,
                       UpperAndLowerVState,
                       PreferredState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorPreferred,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingPreferredArbitrary,
                       UpperAndLowerVStateInfo,
                       PreferredStateInfo,
                       UpperAndLowerVState,
                       PreferredState,
                       t_search_space > > ::create(parser);
#endif
            default:
                break;
            }
        }
#endif

#if COMPILE_AO_OSELECT_GAP
        case GAP: {
            switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
            case action_tie_breaking::ARBITRARY:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorGap,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingArbitrary,
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space > > ::create(parser);
#if COMPILE_TIEBREAKING_MINEXPH
            case action_tie_breaking::MINEXPH:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorGap,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMinExpH,
                       UpperAndLowerVStateInfo,
                       HStateInfo,
                       UpperAndLowerVState,
                       HState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_MINEXPGAP
            case action_tie_breaking::MINEXPGAP:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorGap,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMinExpGap,
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_MAXEXPGAP
            case action_tie_breaking::MAXEXGAP:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorGap,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMaxExpGap,
                       UpperAndLowerVStateInfo,
                       BasicStateInfo,
                       UpperAndLowerVState,
                       BasicState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorGap,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingPreferredArbitrary,
                       UpperAndLowerVStateInfo,
                       PreferredStateInfo,
                       UpperAndLowerVState,
                       PreferredState,
                       t_search_space > > ::create(parser);
#endif
            default:
                break;
            }
        }
#endif

#if COMPILE_AO_OSELECT_MIN_H
        case MIN_H: {
            switch (action_tie_breaking::Type(opts.get_enum("tiebreaking"))) {
            case action_tie_breaking::ARBITRARY:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorMinH,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingArbitrary,
                       UpperAndLowerVStateInfo,
                       HStateInfo,
                       UpperAndLowerVState,
                       HState,
                       t_search_space > > ::create(parser);
#if COMPILE_TIEBREAKING_MINEXPH
            case action_tie_breaking::MINEXPH:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorMinH,
                       t_vinit,
                       t_vreport,
                       StoreHOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMinExpH,
                       UpperAndLowerVStateInfo,
                       HStateInfo,
                       UpperAndLowerVState,
                       HState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_MINEXPGAP
            case action_tie_breaking::MINEXPGAP:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorMinH,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMinExpGap,
                       UpperAndLowerVStateInfo,
                       HStateInfo,
                       UpperAndLowerVState,
                       HState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_MAXEXPGAP
            case action_tie_breaking::MAXEXGAP:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorMinH,
                       t_vinit,
                       t_vreport,
                       DoNothingOnNewSuccessor,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingMaxExpGap,
                       UpperAndLowerVStateInfo,
                       HStateInfo,
                       UpperAndLowerVState,
                       HState,
                       t_search_space > > ::create(parser);
#endif
#if COMPILE_TIEBREAKING_PREFERRED
            case action_tie_breaking::PREFERRED:
                return Factory < AOSearchFactory <
                       AOOutcomeSelectorMinH,
                       t_vinit,
                       t_vreport,
                       StorePref,
                       PaSelect,
                       LabelSolvedUpdateHandler,
                       TieBreakingArbitrary,
                       UpperAndLowerVStateInfo,
                       PreferredHStateInfo,
                       UpperAndLowerVState,
                       PreferredHState,
                       t_search_space > > ::create(parser);
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

    std::cerr << "Unsupported AO* parameter encountered." << std::endl
              << "Make sure that the constants are set correctly in " << std::endl
              << "  prob_search_engine_factory.h" << std::endl
              << "  ao_factory.h" << std::endl
              << "and recompile it." << std::endl;

    return NULL;
}


template<template<typename> class Factory>
static SearchEngine *parse(OptionParser &parser)
{
    add_options_to_parser(parser);
    if (g_use_upper_and_lower) {
        switch (g_optimization_criterion) {
        case OptimizationCriterion::MAXP:
            return generate_dual <
                   ProbSearchSpaceDualMax,
                   BiValueInitializer<OptimizationCriterion::MAXP>,
                   LRTDPBiValueReporter<OptimizationCriterion::MAXP>,
                   Factory,
                   AllParentsSelection > (parser);
#if COMPILE_OPTIMIZATION_CRITERION_EXPC
        case OptimizationCriterion::EXPC:
            return generate_dual <
                   ProbSearchSpaceDualMin,
                   BiValueInitializer<OptimizationCriterion::EXPC>,
                   LRTDPBiValueReporter<OptimizationCriterion::EXPC>,
                   Factory,
                   AllParentsSelection > (parser);
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
                   Factory,
                   AllParentsSelection > (parser);
#if COMPILE_OPTIMIZATION_CRITERION_EXPC
        case OptimizationCriterion::EXPC:
            return generate_upper <
                   ProbSearchSpaceMin,
                   OptimisticValueInitializer<OptimizationCriterion::EXPC>,
                   OptimisticValueReporter<OptimizationCriterion::EXPC>,
                   Factory,
                   AllParentsSelection > (parser);
#endif
        default:
            break;
        }
    }
    std::cerr << "Unsupported optimization criterion." << std::endl
              << "Make sure that the constants are set correctly in " << std::endl
              << "  prob_search_engine_factory.h" << std::endl
              << "  ao_factory.h" << std::endl
              << "and recompile it." << std::endl;

    return NULL;
}

}


namespace prob_search_engine_factory
{
Plugin<SearchEngine> ao("ao", ao_factory::parse<ProbabilisticSearchEngine>);
}

