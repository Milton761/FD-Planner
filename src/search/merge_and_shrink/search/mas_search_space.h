
#ifndef MAS_SEARCH_SPACE_H
#define MAS_SEARCH_SPACE_H

#include "mas_search_node_info.h"
#include "../../segmented_vector.h"
#include <unordered_map>

class MASSearchNode
{
private:
    unsigned id;
public:
    MASSearchNodeInfo &info;
    MASSearchNode(unsigned id, MASSearchNodeInfo &info) : id(id), info(info) {}
    unsigned get_id() const
    {
        return id;
    }
    const MASSearchNodeInfo &get_data() const
    {
        return info;
    }
    MASSearchNodeInfo &get_data()
    {
        return info;
    }
};

class MASSearchSpace
{
private:
    SegmentedVector<MASSearchNodeInfo> data;
    std::vector<std::unordered_map<int, unsigned> > state_id_mapping;
    unsigned currentid;
public:
    MASSearchSpace(unsigned size);
    MASSearchNode get(int state, int budget);
    MASSearchNode operator[](unsigned id);
    size_t size() const
    {
        return currentid;
    }
    float get_value(int state, int budget)
    {
        return get(state, budget).info.value;
    }
};

#endif

