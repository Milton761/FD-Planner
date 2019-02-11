#ifndef MERGE_CRITERIA_H
#define MERGE_CRITERIA_H

#include "scc.h"

#include <list>
#include <vector>
#include <string>
#include <limits>

class OptionParser;
class Options;
class TransitionSystem;

enum MergeOrder {
    MO_LEVEL = 0,
    MO_REVERSE_LEVEL = 1,
    MO_RANDOM = 2,
};


// The new merge strategy is based on a list of criteria.  We start
// with the set of candidate variables and apply each criterion, that
// discards some variables, until only one variable is left. If more
// than one variable is left after applying all the criteria, the
// merge_order is used as final tie-breaking.
//
// CG: prefer v to v' if v has causal influence over already
// merged variables and v' not.
//
// GOAL: prefer goal variables to non-goal variables
//
// Relevant: prefer relevant variables to non-relevant variables. A
// variable is relevant if a) it is a goal or b) it has a causal
// influence over already merged variables.
//
// MinSCC: Same as CG, but applying tie-breaking in case that more
// than one variable is causally relevant: prefer variable v to v' if
// SCC(v) has a path to SCC(v') if SCC(v) has a path to SCC(v').
// Optionally, only one variable is selected per SCC
// (the one with smallest "level", i.e. closer to the SCC root)
//
// Empty: prefer variables that will make more labels of the current
// abstractionempty
//
// Num: prefer variables that apper in more labels of the current
// abstraction.

class MergeCriterion
{
protected:
    //Returns true in case that at least one variable fits the criterion
    bool filter(std::vector <int> &vars,
                const std::vector<bool> &criterion) const {
        std::vector<int> aux;
        for (size_t i = 0; i < vars.size(); i++) {
            if (criterion[vars[i]]) {
                aux.push_back(vars[i]);
            }
        }
        if (!aux.empty()) {
            vars.swap(aux);
            return true;
        }
        return false;
    }

    template<class T>
    void filter_best(std::vector <int> &vars,
                     const std::vector<T> &criterion,
                     bool minimize) const {
        std::vector<int> aux;
        double best = (minimize ? std::numeric_limits<double>::max() : 0.0);
        for (size_t i = 0; i < vars.size(); i++) {
            double score = criterion[vars[i]];
            if ((minimize && score < best) ||
                (!minimize && score > best)) {
                std::vector<int>().swap(aux);
                best = score;
            }
            if (score == best) {
                aux.push_back(vars[i]);
            }
        }
        if (!aux.empty()) {
            vars.swap(aux);
        }
    }

    bool allow_incremental;
public:
    MergeCriterion(const Options &) : allow_incremental(true) {}
    virtual void init() = 0;
    virtual void disable_incremental() {
        allow_incremental = false;
    }
    // Allows for incremental computation (currently used to compute
    // predecessor variables in the CG). However, it should only work if
    // we have no disabled this.
    virtual void select_next(int var_no) = 0;
    virtual void filter(std::vector <int> &vars,
                        TransitionSystem *abstraction)  = 0;
    virtual std::string get_name() const = 0;
    virtual bool reduce_labels_before_merge() const {
        return false;
    }
};

class MergeCriterionCG : public MergeCriterion
{
protected:
    std::vector<bool> is_causal_predecessor;
public:
    MergeCriterionCG(const Options &opts) : MergeCriterion(opts) {}
    virtual void init();
    virtual void select_next(int var_no);
    virtual void filter(std::vector <int> &vars, TransitionSystem *abstraction) ;
    virtual std::string get_name() const {
        return "CG";
    }
};

class MergeCriterionGoal : public MergeCriterion
{
    std::vector<bool> is_goal_variable;
public:
    MergeCriterionGoal(const Options &opts) : MergeCriterion(opts) {}
    virtual void init();
    virtual void select_next(int var_no);
    virtual void filter(std::vector <int> &vars, TransitionSystem *abstraction) ;
    virtual std::string get_name() const {
        return "GOAL";
    }
};

class MergeCriterionRelevant : public MergeCriterionCG
{
public:
    MergeCriterionRelevant(const Options &opts) : MergeCriterionCG(opts) {}
    virtual void init();
    virtual std::string get_name() const {
        return "RELEVANT";
    }
};

class MergeCriterionMinSCC : public MergeCriterion
{
    const bool reverse;
    const bool tie_by_level;

    std::vector<bool> is_causal_predecessor;
    SCC scc;

    void forbid_scc_descendants(int scc_index,
                                const vector<set<int> > &scc_graph,
                                vector<bool> &forbidden_sccs) const;
public:
    MergeCriterionMinSCC(const Options &opts);
    virtual void init();
    virtual void select_next(int var_no);
    virtual void filter(std::vector <int> &vars, TransitionSystem *abstraction) ;
    virtual std::string get_name() const {
        return "SCC";
    }
};

class MergeCriterionEmpty : public MergeCriterion
{
public:
    MergeCriterionEmpty(const Options &opts) : MergeCriterion(opts) {}
    virtual void init();
    virtual void select_next(int var_no);
    virtual void filter(std::vector <int> &vars, TransitionSystem *abstraction) ;
    virtual std::string get_name() const {
        return "EMPTY";
    }
    virtual bool reduce_labels_before_merge() const {
        return true;
    }
};

class MergeCriterionEmptyGoal : public MergeCriterion
{
public:
    MergeCriterionEmptyGoal(const Options &opts) : MergeCriterion(opts) {}
    virtual void init();
    virtual void select_next(int var_no);
    virtual void filter(std::vector <int> &vars, TransitionSystem *abstraction) ;
    virtual std::string get_name() const {
        return "EMPTYGOAL";
    }
    virtual bool reduce_labels_before_merge() const {
        return true;
    }
};


class MergeCriterionNum : public MergeCriterion
{
public:
    MergeCriterionNum(const Options &opts) : MergeCriterion(opts) {}
    virtual void init();
    virtual void select_next(int var_no);
    virtual void filter(std::vector <int> &vars, TransitionSystem *abstraction) ;
    virtual std::string get_name() const {
        return "NUM";
    }
    virtual bool reduce_labels_before_merge() const {
        return true;
    }
};

//// New class to make multiple abstractions in a DFS manner.
//// Works similar to LinearMergeStrategy except that:
////   a) Returns a list of variables instead of a single var.
////       The list is ordered by preference.
////   b) It does not perform incremental computations:
////       (because we may remove variables)
//class MultipleLinearMergeStrategy : public MergeCriterion
//{
//    const std::vector <MergeCriterion *> criteria;
//    const MergeOrder order;
//    std::list<std::vector<int> > remaining_vars;
//    void order_vars(TransitionSystem *abstraction,
//                    std::vector<int> &selected_vars) const;
//public:
//    MultipleLinearMergeStrategy(const std::vector <MergeCriterion *>
//                                &merge_criteria,
//                                const MergeOrder &merge_order, bool is_first);
//    virtual ~MultipleLinearMergeStrategy();
//    virtual void select_next(int var_no);
//    virtual void filter(std::vector <int> &vars,
//                        TransitionSystem *abstraction);
//    virtual std::string get_name() const
//    {
//        return "MULTI";
//    }
//};


#endif
