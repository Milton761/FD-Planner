
#ifndef PROB_SEARCH_ENGINE_LOAD_H
#define PROB_SEARCH_ENGINE_LOAD_H

#include "search_engine.h"
#include "option_parser.h"

#include "optimization_criteria.h"
#include "prob_search_space.h"
#include "state_space_factory.h"
#include "state_space.h"
#include "mas_state_space.h"

#include "prob_search_engine.h"

#include "utilities.h"
#include "globals.h"
#include "plugin.h"

#include <list>

#define COMPILE_TIEBREAKING_MINEXPH 0
#define COMPILE_TIEBREAKING_MINEXPGAP 0
#define COMPILE_TIEBREAKING_MAXEXPGAP 0
#define COMPILE_TIEBREAKING_PREFERRED 0

#define COMPILE_OPTIMIZATION_CRITERION_EXPC 1

namespace prob_search_engine_factory
{

template<typename Factory>
SearchEngine *_parse_prob_search_engine(Factory f,
        OptionParser &parser)
{
    prob_search_engine::add_options_to_parser(parser);
    Options opts = parser.parse();
    if (!parser.dry_run()) {
        state_space_factory::StateSpaceFactory *sp_fact =
            opts.get<state_space_factory::StateSpaceFactory *>("state_space");
        typedef typename Factory::StateInfo A;
        typedef typename Factory::template State<A> B;
        switch (sp_fact->get_type()) {
        case state_space_factory::FACTORED: {
            typedef typename Factory::template
            SearchSpace<StateSpaceLookup, A, Factory::template State> C;
            typedef StateSpaceLookup<B, typename C::StateIDLookup> StateSpace;
            opts.set<StateSpace *>("state_space", new StateSpace(sp_fact->get_options()));
            return f.template create<StateSpaceLookup>(opts);
        }
        case state_space_factory::MERGE_AND_SHRINK: {
            typedef typename Factory::template
            SearchSpace<MASStateSpace, A, Factory::template State> C;
            typedef MASStateSpace<B, typename C::StateIDLookup> StateSpace;
            opts.set<StateSpace *>("state_space", new StateSpace(sp_fact->get_options()));
            return f.template create<MASStateSpace>(opts);
        }
        default:
            cerr << "unsopported state space" <<    endl;
            exit_with(EXIT_CRITICAL_ERROR);
            break;
        }
        return NULL;

    }
    return NULL;
}


//////////////////////////////////////////////////////////////////////////

template<class Factory>
struct ProbabilisticSearchEngine {
    static SearchEngine *create(OptionParser &parser) {
        return _parse_prob_search_engine(Factory(),
                                         parser);
    }
};

template<class Factory>
SearchEngine *_parse(OptionParser &parser) {
    return ProbabilisticSearchEngine<Factory>::create(parser);
}

}

#endif

