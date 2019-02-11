#ifndef SEARCH_ENGINE_H
#define SEARCH_ENGINE_H

#include <vector>

class Heuristic;
class OptionParser;
class Options;

#include "global_operator.h"
#include "operator_cost.h"
#include "search_progress.h"
#include "search_space.h"
#include "search_statistics.h"

enum SearchStatus {IN_PROGRESS, TIMEOUT, FAILED, SOLVED};

class AbstractTask;

class SearchEngine
{
protected:
    SearchStatus status;
    bool solution_found;

    std::shared_ptr<AbstractTask> atask;
    double max_time;

    virtual void initialize() {}

    float get_adjusted_cost(const GlobalOperator &op) const;
    SearchEngine(SearchEngine *copy);
public:
    SearchEngine(const Options &opts);
    virtual ~SearchEngine();
    virtual void print_statistics() const;
    virtual void save_plan_if_necessary() const {}
    bool found_solution() const;
    SearchStatus get_status() const;
    virtual void search() = 0;
    virtual void set_bound(int /*b*/) { }
    virtual int get_bound() {
        return -1;
    }
    static void add_options_to_parser(OptionParser &parser);
};

#endif
