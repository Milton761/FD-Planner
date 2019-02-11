#include "legacy_causal_graph.h"

#include "globals.h"
#include "global_operator.h"

using namespace std;

LegacyCausalGraph::LegacyCausalGraph()
{
    vector<vector<bool> > connected;
    connected.resize(g_variable_domain.size());
    for (size_t i = 0; i < g_variable_domain.size(); i++) {
        connected[i].resize(g_variable_domain.size(), false);
    }
    for (size_t i = 0; i < g_operators.size(); i++) {
        const GlobalOperator &op = g_operators[i];
        for (size_t j = 0; j < op.get_preconditions().size(); j++) {
            for (size_t k = 0; k < op.get_effects().size(); k++) {
                connected[op.get_preconditions()[j].var][op.get_effects()[k].var] = true;
            }
        }
    }
    arcs.resize(g_variable_domain.size());
    inverse_arcs.resize(g_variable_domain.size());
    //edges.resize(g_variable_domain.size());
    for (size_t i = 0; i < connected.size(); i++) {
        for (size_t j = 0; j < connected[i].size(); j++) {
            if (i != j && connected[i][j]) {
                arcs[i].push_back(j);
                inverse_arcs[i].push_back(j);
            }
        }
    }
    connected.clear();
}

const vector<int> &LegacyCausalGraph::get_successors(int var) const
{
    return arcs[var];
}

const vector<int> &LegacyCausalGraph::get_predecessors(int var) const
{
    return inverse_arcs[var];
}

//const vector<int> &LegacyCausalGraph::get_neighbours(int var) const {
//    return edges[var];
//}

void LegacyCausalGraph::dump() const
{
    cout << "Causal graph: " << endl;
    for (size_t i = 0; i < arcs.size(); i++) {
        cout << "dependent on var " << g_variable_name[i] << ": " << endl;
        for (size_t j = 0; j < arcs[i].size(); j++) {
            cout << "  " << g_variable_name[arcs[i][j]] << ",";
        }
        cout << endl;
    }
}
