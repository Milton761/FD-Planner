#ifndef SEARCH_SPACE_H
#define SEARCH_SPACE_H

#include "global_state.h"
#include "operator_cost.h"
#include "per_state_information.h"
#include "search_node_info.h"

#include <vector>

class GlobalOperator;
class GlobalState;


class SearchNode
{
    StateID state_id;
    SearchNodeInfo &info;
public:
    SearchNode(StateID state_id_, SearchNodeInfo &info_);

    StateID get_state_id() const {
        return state_id;
    }
    GlobalState get_state() const;

    bool is_new() const;
    bool is_open() const;
    bool is_closed() const;
    bool is_dead_end() const;

    bool is_h_dirty() const;
    void set_h_dirty();
    void clear_h_dirty();
    float get_g() const;
    float get_real_g() const;
    float get_h() const;

    void open_initial(float h);
    void open(float h, const SearchNode &parent_node,
              const GlobalOperator *parent_op, float cost);
    void reopen(const SearchNode &parent_node,
                const GlobalOperator *parent_op, float cost);
    void update_parent(const SearchNode &parent_node,
                       const GlobalOperator *parent_op, float cost);
    void increase_h(float h);
    void close();
    void mark_as_dead_end();

    void dump() const;
};


class SearchSpace
{
    PerStateInformation<SearchNodeInfo> search_node_infos;
public:
    SearchSpace();
    SearchNode get_node(const GlobalState &state);
    void trace_path(const GlobalState &goal_state,
                    std::vector<const GlobalOperator *> &path) const;

    void dump() const;
    void print_statistics() const;
};

#endif
