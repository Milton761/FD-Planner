
#ifndef AO_TIP_LIST_H
#define AO_TIP_LIST_H

#include <cstdlib>
#include <deque>
#include <map>

#include "template_factory.h"
#include "option_parser.h"
#include "plugin.h"

#include "heuristic.h"

namespace ao_tip_list
{
enum Type {
    LIFO = 0,
    UPPERLOWER = 1,
    FIFO = 2,
    DISTANCE_ASC = 3,
    HEURISTIC_ASC = 4,
    HEURISTIC_DESC = 5,
    DISTANCE_DESC = 6,
    DD = 7,
    BUDGET_ASC = 8,
    DDd = 9,
    BUDGET_DESC = 10,
};

struct TipListFactory : public TemplateFactory {
    virtual Type get_type() const {
        return LIFO;
    }
};

/* ********************************************** */
/* FIFO */

template<typename NodeInfo, template<typename> class Node>
struct FIFOTipList {
    typedef Node<NodeInfo> State;
private:
    std::deque<size_t> sid;
public:
    FIFOTipList(const Options &) {}
    inline void insert(const State &s) {
        sid.push_back(s.get_prob_state_id());
    }
    inline void clear() {
        sid.clear();
    }
    inline size_t next() {
        size_t res = sid.front();
        sid.pop_front();
        return res;
    }
    inline bool empty() const {
        return sid.empty();
    }
    inline void initialize() {}
    inline bool readd() const { return false; }
};

struct FIFOTipListFactory : public TipListFactory {
public:
    FIFOTipListFactory(const Options &) {
    }
    virtual Type get_type() const {
        return FIFO;
    }
    static TipListFactory *_parse(OptionParser &parser) {
        Options opts = parser.parse();
        if (!parser.dry_run()) {
            return new FIFOTipListFactory(opts);
        }
        return NULL;
    }
};

static Plugin<TipListFactory> _plugin_fifo("fifo", FIFOTipListFactory::_parse);

/* ********************************************** */
/* LIFO */

template<typename NodeInfo, template<typename> class Node>
struct LIFOTipList {
    typedef Node<NodeInfo> State;
private:
    std::deque<size_t> sid;
public:
    LIFOTipList(const Options &) {}
    inline void insert(const State &s) {
        sid.push_back(s.get_prob_state_id());
    }
    inline void clear() {
        sid.clear();
    }
    inline size_t next() {
        size_t res = sid.back();
        sid.pop_back();
        return res;
    }
    inline bool empty() const {
        return sid.empty();
    }
    inline void initialize() {}
    inline bool readd() const { return true; }
};

struct LIFOTipListFactory : public TipListFactory {
public:
    LIFOTipListFactory(const Options &) {
    }
    virtual Type get_type() const {
        return LIFO;
    }
    static TipListFactory *_parse(OptionParser &parser) {
        Options opts = parser.parse();
        if (!parser.dry_run()) {
            return new LIFOTipListFactory(opts);
        }
        return NULL;
    }
};

static Plugin<TipListFactory> _plugin_lifo("lifo", LIFOTipListFactory::_parse);

/* ********************************************** */
/* UPPERLOWER */

template<typename NodeInfo, template<typename> class Node>
struct ULTipList {
    typedef Node<NodeInfo> State;
private:
    typedef std::deque<size_t> Bucket;
    std::map<float, Bucket> sid;
public:
    ULTipList(const Options &) {}
    inline void insert(const State &s) {
        sid[fabs(s.get_v_current() - s.get_v_optimistic())].push_back(s.get_prob_state_id());
    }
    inline void clear() {
        sid.clear();
    }
    inline size_t next() {
        std::map<float, Bucket>::reverse_iterator it = sid.rbegin();
        size_t res = it->second.back();
        it->second.pop_back();
        if (it->second.empty()) {
            sid.erase(--(it.base()));
        }
        return res;
    }
    inline bool empty() const {
        return sid.empty();
    }
    inline void initialize() {}
    inline bool readd() const { return false; }
};

struct ULTipListFactory : public TipListFactory {
public:
    ULTipListFactory(const Options &) {
    }
    virtual Type get_type() const {
        return UPPERLOWER;
    }
    static TipListFactory *_parse(OptionParser &parser) {
        // todo add ip
        Options opts = parser.parse();
        if (!parser.dry_run()) {
            return new ULTipListFactory(opts);
        }
        return NULL;
    }
};

static Plugin<TipListFactory> _plugin_ul("upperlower", ULTipListFactory::_parse);

/* ********************************************** */
/* DISTANCE_ASC */

template<typename NodeInfo, template<typename> class Node>
struct DistanceTipList {
    typedef Node<NodeInfo> State;
private:
    typedef std::deque<size_t> Bucket;
    std::map<float, Bucket> sid;
public:
    DistanceTipList(const Options &) {}
    inline void insert(const State &s) {
        sid[s.get_estimated_distance()].push_back(s.get_prob_state_id());
    }
    inline void clear() {
        sid.clear();
    }
    inline size_t next() {
        std::map<float, Bucket>::iterator it = sid.begin();
        size_t res = it->second.back();
        it->second.pop_back();
        if (it->second.empty()) {
            sid.erase(it);
        }
        return res;
    }
    inline bool empty() const {
        return sid.empty();
    }
    inline void initialize() {}
    inline bool readd() const { return false; }
};

struct DistanceTipListFactory : public TipListFactory {
public:
    DistanceTipListFactory(const Options &) {
    }
    virtual Type get_type() const {
        return DISTANCE_ASC;
    }
    static TipListFactory *_parse(OptionParser &parser) {
        Options opts = parser.parse();
        if (!parser.dry_run()) {
            return new DistanceTipListFactory(opts);
        }
        return NULL;
    }
};

static Plugin<TipListFactory> _plugin_distasc("distasc", DistanceTipListFactory::_parse);

/* ********************************************** */
/* DDd */

template<typename NodeInfo, template<typename> class Node>
struct DDdTipList {
    typedef Node<NodeInfo> State;
private:
    typedef std::deque<size_t> Bucket;
    std::map<int, std::map<float, Bucket> > sid;
public:
    DDdTipList(const Options &) {}
    inline void insert(const State &s) {
        sid[s.get_depth()][s.get_estimated_distance()].push_back(s.get_prob_state_id());
    }
    inline void clear() {
        sid.clear();
    }
    inline size_t next() {
        std::map<int, std::map<float, Bucket> >::reverse_iterator it = sid.rbegin();
        std::map<float, Bucket>::reverse_iterator it2 = it->second.rbegin();
        size_t res = it2->second.back();
        it2->second.pop_back();
        if (it2->second.empty()) {
            it->second.erase(--(it2.base()));
            if (it->second.empty()) {
                sid.erase(--(it.base()));
            }
        }
        return res;
    }
    inline bool empty() const {
        return sid.empty();
    }
    inline void initialize() {}
    inline bool readd() const { return true; }
};

struct DDdTipListFactory : public TipListFactory {
public:
    DDdTipListFactory(const Options &) {
    }
    virtual Type get_type() const {
        return DDd;
    }
    static TipListFactory *_parse(OptionParser &parser) {
        Options opts = parser.parse();
        if (!parser.dry_run()) {
            return new DDdTipListFactory(opts);
        }
        return NULL;
    }
};

static Plugin<TipListFactory> _plugin_ddd("ddd", DDdTipListFactory::_parse);

/* ********************************************** */
/* DD */

template<typename NodeInfo, template<typename> class Node>
struct DDTipList {
    typedef Node<NodeInfo> State;
private:
    typedef std::deque<size_t> Bucket;
    std::map<int, std::map<float, Bucket> > sid;
public:
    DDTipList(const Options &) {}
    inline void insert(const State &s) {
        sid[s.get_depth()][s.get_estimated_distance()].push_back(s.get_prob_state_id());
    }
    inline void clear() {
        sid.clear();
    }
    inline size_t next() {
        std::map<int, std::map<float, Bucket> >::reverse_iterator it = sid.rbegin();
        std::map<float, Bucket>::iterator it2 = it->second.begin();
        size_t res = it2->second.back();
        it2->second.pop_back();
        if (it2->second.empty()) {
            it->second.erase(it2);
            if (it->second.empty()) {
                sid.erase(--(it.base()));
            }
        }
        return res;
    }
    inline bool empty() const {
        return sid.empty();
    }
    inline void initialize() {}
    inline bool readd() const { return true; }
};

struct DDTipListFactory : public TipListFactory {
public:
    DDTipListFactory(const Options &) {
    }
    virtual Type get_type() const {
        return DD;
    }
    static TipListFactory *_parse(OptionParser &parser) {
        Options opts = parser.parse();
        if (!parser.dry_run()) {
            return new DDTipListFactory(opts);
        }
        return NULL;
    }
};

static Plugin<TipListFactory> _plugin_dd("dd", DDTipListFactory::_parse);

/* ********************************************** */
/* DISTANCE_DESC */

template<typename NodeInfo, template<typename> class Node>
struct DistanceDescTipList {
    typedef Node<NodeInfo> State;
private:
    typedef std::deque<size_t> Bucket;
    std::map<float, Bucket> sid;
public:
    DistanceDescTipList(const Options &) {}
    inline void insert(const State &s) {
        sid[s.get_estimated_distance()].push_back(s.get_prob_state_id());
    }
    inline void clear() {
        sid.clear();
    }
    inline size_t next() {
        std::map<float, Bucket>::reverse_iterator it = sid.rbegin();
        size_t res = it->second.back();
        it->second.pop_back();
        if (it->second.empty()) {
            sid.erase(--(it.base()));
        }
        return res;
    }
    inline bool empty() const {
        return sid.empty();
    }
    inline void initialize() {}
    inline bool readd() const { return false; }
};

struct DistanceDescTipListFactory : public TipListFactory {
public:
    DistanceDescTipListFactory(const Options &) {
    }
    virtual Type get_type() const {
        return DISTANCE_DESC;
    }
    static TipListFactory *_parse(OptionParser &parser) {
        Options opts = parser.parse();
        if (!parser.dry_run()) {
            return new DistanceDescTipListFactory(opts);
        }
        return NULL;
    }
};

static Plugin<TipListFactory> _plugin_distdesc("distdesc", DistanceDescTipListFactory::_parse);


/* ********************************************** */
/* HEURISTIC_ASC */

template<typename NodeInfo, template<typename> class Node>
struct MinHeuristicTipList {
    typedef Node<NodeInfo> State;
private:
    typedef std::deque<size_t> Bucket;
    std::map<float, Bucket> sid;
public:
    MinHeuristicTipList(const Options &) {}
    inline void insert(const State &s) {
        sid[s.get_v_current()].push_back(s.get_prob_state_id());
    }
    inline void clear() {
        sid.clear();
    }
    inline size_t next() {
        std::map<float, Bucket>::iterator it = sid.begin();
        size_t res = it->second.back();
        it->second.pop_back();
        if (it->second.empty()) {
            sid.erase(it);
        }
        return res;
    }
    inline bool empty() const {
        return sid.empty();
    }
    inline void initialize() {}
    inline bool readd() const { return false; }
};

struct MinHeuristicTipListFactory : public TipListFactory {
public:
    MinHeuristicTipListFactory(const Options &) {
    }
    virtual Type get_type() const {
        return HEURISTIC_ASC;
    }
    static TipListFactory *_parse(OptionParser &parser) {
        Options opts = parser.parse();
        if (!parser.dry_run()) {
            return new MinHeuristicTipListFactory(opts);
        }
        return NULL;
    }
};

static Plugin<TipListFactory> _plugin_minh("hasc", MinHeuristicTipListFactory::_parse);

/* ********************************************** */
/* HEURISTIC_DESC */

template<typename NodeInfo, template<typename> class Node>
struct MaxHeuristicTipList {
    typedef Node<NodeInfo> State;
private:
    typedef std::deque<size_t> Bucket;
    std::map<float, Bucket> sid;
public:
    MaxHeuristicTipList(const Options &) {}
    inline void insert(const State &s) {
        sid[s.get_v_current()].push_back(s.get_prob_state_id());
    }
    inline void clear() {
        sid.clear();
    }
    inline size_t next() {
        std::map<float, Bucket>::reverse_iterator it = sid.rbegin();
        size_t res = it->second.back();
        it->second.pop_back();
        if (it->second.empty()) {
            sid.erase(--(it.base()));
        }
        return res;
    }
    inline bool empty() const {
        return sid.empty();
    }
    inline void initialize() {}
    inline bool readd() const { return false; }
};

struct MaxHeuristicTipListFactory : public TipListFactory {
public:
    MaxHeuristicTipListFactory(const Options &) {
    }
    virtual Type get_type() const {
        return HEURISTIC_DESC;
    }
    static TipListFactory *_parse(OptionParser &parser) {
        // todo add ip
        Options opts = parser.parse();
        if (!parser.dry_run()) {
            return new MaxHeuristicTipListFactory(opts);
        }
        return NULL;
    }
};

static Plugin<TipListFactory> _plugin_maxh("hdesc", MaxHeuristicTipListFactory::_parse);

/* ********************************************** */
/* BUDGET_ASC */

template<typename NodeInfo, template<typename> class Node>
struct BudgetAscTipList {
    typedef Node<NodeInfo> State;
private:
    typedef std::deque<size_t> Bucket;
    std::map<float, Bucket> sid;
public:
    BudgetAscTipList(const Options &) {}
    inline void insert(const State &s) {
        sid[s.get_budget()].push_back(s.get_prob_state_id());
    }
    inline void clear() {
        sid.clear();
    }
    inline size_t next() {
        std::map<float, Bucket>::iterator it = sid.begin();
        size_t res = it->second.back();
        it->second.pop_back();
        if (it->second.empty()) {
            sid.erase(it);
        }
        return res;
    }
    inline bool empty() const {
        return sid.empty();
    }
    inline void initialize() {}
    inline bool readd() const { return false; }
};

struct BudgetAscTipListFactory : public TipListFactory {
public:
    BudgetAscTipListFactory(const Options &) {
    }
    virtual Type get_type() const {
        return BUDGET_ASC;
    }
    static TipListFactory *_parse(OptionParser &parser) {
        // todo add ip
        Options opts = parser.parse();
        if (!parser.dry_run()) {
            return new BudgetAscTipListFactory(opts);
        }
        return NULL;
    }
};

static Plugin<TipListFactory> _plugin_basc("basc", BudgetAscTipListFactory::_parse);

template<typename NodeInfo, template<typename> class Node>
struct BudgetDescTipList {
    typedef Node<NodeInfo> State;
private:
    typedef std::deque<size_t> Bucket;
    std::map<float, Bucket> sid;
public:
    BudgetDescTipList(const Options &) {}
    inline void insert(const State &s) {
        sid[s.get_budget()].push_back(s.get_prob_state_id());
    }
    inline void clear() {
        sid.clear();
    }
    inline size_t next() {
        std::map<float, Bucket>::reverse_iterator it = sid.rbegin();
        size_t res = it->second.back();
        it->second.pop_back();
        if (it->second.empty()) {
            sid.erase(--(it.base()));
        }
        return res;
    }
    inline bool empty() const {
        return sid.empty();
    }
    inline void initialize() {}
    inline bool readd() const { return false; }
};

struct BudgetDescTipListFactory : public TipListFactory {
public:
    BudgetDescTipListFactory(const Options &) {
    }
    virtual Type get_type() const {
        return BUDGET_DESC;
    }
    static TipListFactory *_parse(OptionParser &parser) {
        // todo add ip
        Options opts = parser.parse();
        if (!parser.dry_run()) {
            return new BudgetDescTipListFactory(opts);
        }
        return NULL;
    }
};

static Plugin<TipListFactory> _plugin_bdesc("bdesc", BudgetDescTipListFactory::_parse);

}

#include "ao_tip_list.cc"

#endif
