#ifndef MAS_SEARCH_NODE_INFO_H
#define MAS_SEARCH_NODE_INFO_H

#include <vector>
#include <unordered_set>

struct MASSearchNodeInfo
{
    enum {
        NEW = 0, GOAL = 1, DEAD_END = 2, DEFAULT = 3
    };
    unsigned status : 2;
    int budget : 20;
    float value;
    //std::vector<std::vector<unsigned> > successors;
    //std::vector<int> operators;
    std::unordered_set<unsigned> parents;
    int policy;
    int sid;
    unsigned unsolved_successors;

    MASSearchNodeInfo(int budget, int sid) : status(NEW), budget(budget), value(0), policy(-1), sid(sid), unsolved_successors(0) {}

    bool is_new() const
    {
        return status == NEW;
    }
    bool is_goal() const
    {
        return status == GOAL;
    }
    bool is_dead_end() const
    {
        return status == DEAD_END;
    }
    bool is_default() const
    {
        return status == DEFAULT;
    }
    void set_seen()
    {
        status = DEFAULT;
    }
    void set_goal()
    {
        status = GOAL;
    }
    void set_dead_end()
    {
        status = DEAD_END;
    }
};

#endif
