
#ifndef FRET_FACTORY_H
#define FRET_FACTORY_H

#include "prob_search_engine_factory.h"

#include "fret.h"
#include "lrtdp_factory.h"
#include "tarjan_dfs_factory.h"

namespace fret_factory
{
struct FRETEngine {
    virtual SearchEngine *generate(OptionParser &parser) = 0;
};

template<typename Factory, template<int Type> class Vreport>
struct FRETFactory {
    Factory f;

    using StateInfo = typename Factory::StateInfo;
    template<typename Info>
    using State = typename Factory::template State<Info>;
    template<template<typename, typename> class A, typename B, template<typename> class C>
    using SearchSpace = typename Factory::template SearchSpace<A, B, C>;
    using Vinit = typename Factory::Vinit;
    using TieBreaking = typename Factory::TieBreaking;


    template<template<typename, typename> class StateSpace>
    SearchEngine *create(Options &opts)  {
        switch (g_optimization_criterion) {
        case OptimizationCriterion::MAXP:
            opts.set<SearchEngine *>("engine",
                                     f.create<StateInfo, State, SearchSpace, StateSpace, Vinit, Vreport<OptimizationCriterion::MAXP>, TieBreaking>
                                     (opts));
            return new
                   FRET<StateInfo, State, SearchSpace, StateSpace, Vinit, Vreport<OptimizationCriterion::MAXP>, TieBreaking>
                   (opts);
#if COMPILE_OPTIMIZATION_CRITERION_EXPC
        case OptimizationCriterion::EXPC:
            opts.set<SearchEngine *>("engine",
                                     f.create<StateInfo, State, SearchSpace, StateSpace, Vinit, Vreport<OptimizationCriterion::EXPC>, TieBreaking>
                                     (opts));
            return new
                   FRET<StateInfo, State, SearchSpace, StateSpace, Vinit, Vreport<OptimizationCriterion::EXPC>, TieBreaking>
                   (opts);
#endif
        default:
            break;
        }
        std::cerr << "Unsupported optimization criterion." << std::endl
                  << "Make sure that the constants are set correctly in " << std::endl
                  << "  prob_search_engine_factory.h" << std::endl
                  << "  fret_factory.h" << std::endl
                  << "and recompile it." << std::endl;
        return NULL;
    }
};

template<typename Factory>
struct FRETCombinationUpper
        : public prob_search_engine_factory::ProbabilisticSearchEngine<FRETFactory<Factory, OptimisticValueReporter> > {};

template<typename Factory>
struct FRETCombinationDual
        : public prob_search_engine_factory::ProbabilisticSearchEngine<FRETFactory<Factory, FRETLRTDPBiValueReporter> > {};

struct LRTDPFRETEngine : public FRETEngine {
    Options opts;
    LRTDPFRETEngine(const Options &opts) : opts(opts) {}
    static FRETEngine *parse(OptionParser &parser) {
        lrtdp_factory::add_options_to_parser(parser);
        Options opts = parser.parse();
        if (parser.dry_run()) {
            return NULL;
        }
        return new LRTDPFRETEngine(opts);
    }
    virtual SearchEngine *generate(OptionParser &parser) {
        lrtdp_factory::copy_options_to_parser(opts, parser);
        if (g_use_upper_and_lower) {
            return lrtdp_factory::parse_dual<FRETCombinationDual>(parser);
        } else {
            return lrtdp_factory::parse_single<FRETCombinationUpper>(parser);
        }
    }
};

struct DFSFRETEngine : public FRETEngine {
    Options opts;
    DFSFRETEngine(const Options &opts) : opts(opts) {
    }
    static FRETEngine *parse(OptionParser &parser) {
        tarjan_dfs_factory::add_options_to_parser(parser);
        Options opts = parser.parse();
        if (parser.dry_run()) {
            return NULL;
        }
        return new DFSFRETEngine(opts);
    }
    virtual SearchEngine *generate(OptionParser &parser) {
        tarjan_dfs_factory::copy_options_to_parser(opts, parser);
        if (g_use_upper_and_lower) {
            return tarjan_dfs_factory::parse_dual<FRETCombinationDual>(parser);
        } else {
            return tarjan_dfs_factory::parse_single<FRETCombinationUpper>(parser);
        }
    }
};

void add_options_to_parser(OptionParser &parser,
                           const std::string &alg = "lrtdp");
SearchEngine *parse(OptionParser &parser);
}

namespace prob_search_engine_factory
{
extern Plugin<SearchEngine> fret;
}

#endif
