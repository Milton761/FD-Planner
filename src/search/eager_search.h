#ifndef EAGER_SEARCH_H
#define EAGER_SEARCH_H

#include "deterministic_search_engine.h"

#include "open_lists/open_list.h"

#include <vector>

class GlobalOperator;
class Heuristic;
class Options;
class ScalarEvaluator;

class EagerSearch : public DeterministicSearchEngine
{
    const bool reopen_closed_nodes;
    const bool use_multi_path_dependence;

    OpenList<StateID> *open_list;
    ScalarEvaluator *f_evaluator;

    std::vector<Heuristic *> heuristics;
    std::vector<Heuristic *> preferred_operator_heuristics;

    std::pair<SearchNode, bool> fetch_next_node();
    void start_f_value_statistics(EvaluationContext &eval_context);
    void update_f_value_statistics(const SearchNode &node);
    void reward_progress();
    void print_checkpoint_line(float g) const;

protected:
    virtual void reset() override;
    virtual void initialize() override;
    virtual SearchStatus step() override;

public:
    explicit EagerSearch(const Options &opts);
    virtual ~EagerSearch() = default;

    virtual void print_statistics() const override;

    void dump_search_space() const;
};

#endif
