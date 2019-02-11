
#include "mas_search_space.h"

using namespace std;

MASSearchSpace::MASSearchSpace(unsigned size)
{
    state_id_mapping.resize(size);
    currentid = 0;
}

MASSearchNode MASSearchSpace::get(int state, int budget)
{
    unordered_map<int, unsigned> &x = state_id_mapping[state];
    unordered_map<int, unsigned>::iterator it = x.find(budget);
    if (it == x.end()) {
        x[budget] = currentid;
        data.push_back(MASSearchNodeInfo(budget, state));
        return (*this)[currentid++];
    }
    return (*this)[it->second];
}

MASSearchNode MASSearchSpace::operator[](unsigned i)
{
    return MASSearchNode(i, data[i]);
}

