
#ifndef LEGACY_CAUSAL_GRAPH_H
#define LEGACY_CAUSAL_GRAPH_H

#include <vector>

class LegacyCausalGraph
{
    std::vector<std::vector<int> > arcs;
    std::vector<std::vector<int> > inverse_arcs;
    //std::vector<std::vector<int> > edges;
public:
    LegacyCausalGraph();
    ~LegacyCausalGraph() {}
    const std::vector<int> &get_successors(int var) const;
    const std::vector<int> &get_predecessors(int var) const;
    const std::vector<std::vector<int> > &get_arcs() const {
        return arcs;
    }
    const std::vector<std::vector<int> > &get_inverse_arcs() const {
        return inverse_arcs;
    }
    //const std::vector<int> &get_neighbours(int var) const;
    void dump() const;
};

#endif

