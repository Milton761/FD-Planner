#include "globals.h"

#include "axioms.h"
#include "legacy_causal_graph.h"
#include "domain_transition_graph.h"
#include "global_operator.h"
#include "global_state.h"
#include "heuristic.h"
#include "int_packer.h"
#include "rng.h"
#include "root_task.h"
#include "prob_root_task.h"
#include "state_registry.h"
#include "successor_generator.h"
#include "timer.h"
#include "utilities.h"
#include "prob_search_space.h"

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <numeric>


using namespace std;

static const int PRE_FILE_VERSION = 3;

const float g_epsilon = 0.0000001;
float g_giveup = 1000000;
bool g_store_policy = false;
bool g_use_upper_and_lower = false;

OptimizationCriterion g_optimization_criterion = OptimizationCriterion::MAXP;
int g_budget = -1;

void print_probability(float value, const std::string &name)
{
    printf("V(%s) = %.9f\n", name.c_str(), value);
}

void dump_prob_state(const GlobalState &state, int budget, bool newline, std::ostream &out)
{
    state.dump_pddl(out, false);
    if (g_budget >= 0) {
        out << " budget=" << budget;
    }
    if (newline) {
        out << std::endl;
    }
}

static pair<pair<float, float>, string> parse_operator_outcome_name(
    const string &name)
{
    size_t det_pos = name.find("_DETDUP_");
    if (det_pos != string::npos) {
        // int det_num_pos = det_pos + 8;
        size_t weight_pos = name.find("_WEIGHT_");
        if (weight_pos != string::npos) {
            size_t weight_num_pos = weight_pos + 8;
            string tmp = name.substr(weight_num_pos, string::npos);
            weight_num_pos = tmp.find("_");
            float numerator, denominator;
            istringstream(tmp.substr(0, weight_num_pos)) >> numerator;
            istringstream(tmp.substr(weight_num_pos + 1, string::npos)) >> denominator;
            return make_pair(make_pair(numerator, denominator),
                             name.substr(0, det_pos) + name.substr(name.find(" "),
                                     string::npos));
        } else {
            return make_pair(make_pair(1, 1), name.substr(0,
                             det_pos) + name.substr(name.find(" "), string::npos));
        }
    } else {
        return make_pair(make_pair(1, 1), name);
    }
}

int gcd(int a, int b)
{
    for (;;) {
        if (a == 0) {
            return b;
        }
        b %= a;
        if (b == 0) {
            return a;
        }
        a %= b;
    }
}

int lcm(int a, int b)
{
    int temp = gcd(a, b);

    return temp ? (a / temp * b) : 0;
}


int compute_budget(int current, int cost)
{
    return current == -1 ? -1 : (cost > current ? -2 : current - cost);
}

bool budget_is_sufficient(float budget, float cost)
{
    return budget == -1 || cost <= budget;
}


// TODO: This needs a proper type and should be moved to a separate
//       mutexes.cc file or similar, accessed via something called
//       g_mutexes. (Right now, the interface is via global function
//       are_mutex, which is at least better than exposing the data
//       structure globally.)

static vector<vector<set<pair<int, int> > > > g_inconsistent_facts;

bool test_goal(const GlobalState &state)
{
    for (size_t i = 0; i < g_goal.size(); ++i) {
        if (state[g_goal[i].first] != g_goal[i].second) {
            return false;
        }
    }
    return true;
}

//bool prob_test_goal(const ProbSearchState &state) {
//    return (state.get_budget() == -1 || state.get_budget() >= 0) &&
//        test_goal(state.get_global_state());
//}

int calculate_plan_cost(const vector<const GlobalOperator *> &plan)
{
    // TODO: Refactor: this is only used by save_plan (see below)
    //       and the SearchEngine classes and hence should maybe
    //       be moved into the SearchEngine (along with save_plan).
    int plan_cost = 0;
    for (size_t i = 0; i < plan.size(); ++i) {
        plan_cost += plan[i]->get_cost();
    }
    return plan_cost;
}

void save_plan(const vector<const GlobalOperator *> &plan,
               bool generates_multiple_plan_files)
{
    // TODO: Refactor: this is only used by the SearchEngine classes
    //       and hence should maybe be moved into the SearchEngine.
    ostringstream filename;
    filename << g_plan_filename;
    int plan_number = g_num_previously_generated_plans + 1;
    if (generates_multiple_plan_files || g_is_part_of_anytime_portfolio) {
        filename << "." << plan_number;
    } else {
        assert(plan_number == 1);
    }
    ofstream outfile(filename.str());
    for (size_t i = 0; i < plan.size(); ++i) {
        cout << plan[i]->get_name() << " (" << plan[i]->get_cost() << ")" << endl;
        outfile << "(" << plan[i]->get_name() << ")" << endl;
    }
    int plan_cost = calculate_plan_cost(plan);
    outfile << "; cost = " << plan_cost << " ("
            << (is_unit_cost() ? "unit cost" : "general cost") << ")" << endl;
    outfile.close();
    cout << "Plan length: " << plan.size() << " step(s)." << endl;
    cout << "Plan cost: " << plan_cost << endl;
    ++g_num_previously_generated_plans;
}

bool peek_magic(istream &in, string magic)
{
    string word;
    in >> word;
    bool result = (word == magic);
    for (int i = word.size() - 1; i >= 0; --i) {
        in.putback(word[i]);
        assert(in.good());
    }
    return result;
}

void check_magic(istream &in, string magic)
{
    string word;
    in >> word;
    if (word != magic) {
        cout << "Failed to match magic word '" << magic << "'." << endl;
        cout << "Got '" << word << "'." << endl;
        if (magic == "begin_version") {
            cerr << "Possible cause: you are running the planner "
                 << "on a preprocessor file from " << endl
                 << "an older version." << endl;
        }
        exit_with(EXIT_INPUT_ERROR);
    }
}

void read_and_verify_version(istream &in)
{
    int version;
    check_magic(in, "begin_version");
    in >> version;
    check_magic(in, "end_version");
    if (version != PRE_FILE_VERSION) {
        cerr << "Expected preprocessor file version " << PRE_FILE_VERSION
             << ", got " << version << "." << endl;
        cerr << "Exiting." << endl;
        exit_with(EXIT_INPUT_ERROR);
    }
}

void read_metric(istream &in)
{
    check_magic(in, "begin_metric");
    in >> g_use_metric;
    check_magic(in, "end_metric");
}

void read_variables(istream &in)
{
    int count;
    in >> count;
    for (int i = 0; i < count; ++i) {
        check_magic(in, "begin_variable");
        string name;
        in >> name;
        g_variable_name.push_back(name);
        int layer;
        in >> layer;
        g_axiom_layers.push_back(layer);
        int range;
        in >> range;
        g_variable_domain.push_back(range);
        in >> ws;
        vector<string> fact_names(range);
        for (size_t j = 0; j < fact_names.size(); ++j) {
            getline(in, fact_names[j]);
        }
        g_fact_names.push_back(fact_names);
        check_magic(in, "end_variable");
    }
}

void read_mutexes(istream &in)
{
    g_inconsistent_facts.resize(g_variable_domain.size());
    for (size_t i = 0; i < g_variable_domain.size(); ++i) {
        g_inconsistent_facts[i].resize(g_variable_domain[i]);
    }

    int num_mutex_groups;
    in >> num_mutex_groups;

    /* NOTE: Mutex groups can overlap, in which case the same mutex
       should not be represented multiple times. The current
       representation takes care of that automatically by using sets.
       If we ever change this representation, this is something to be
       aware of. */

    for (int i = 0; i < num_mutex_groups; ++i) {
        check_magic(in, "begin_mutex_group");
        int num_facts;
        in >> num_facts;
        vector<pair<int, int> > invariant_group;
        invariant_group.reserve(num_facts);
        for (int j = 0; j < num_facts; ++j) {
            int var, val;
            in >> var >> val;
            invariant_group.push_back(make_pair(var, val));
        }
        check_magic(in, "end_mutex_group");
        for (size_t j = 0; j < invariant_group.size(); ++j) {
            const pair<int, int> &fact1 = invariant_group[j];
            int var1 = fact1.first, val1 = fact1.second;
            for (size_t k = 0; k < invariant_group.size(); ++k) {
                const pair<int, int> &fact2 = invariant_group[k];
                int var2 = fact2.first;
                if (var1 != var2) {
                    /* The "different variable" test makes sure we
                       don't mark a fact as mutex with itself
                       (important for correctness) and don't include
                       redundant mutexes (important to conserve
                       memory). Note that the preprocessor removes
                       mutex groups that contain *only* redundant
                       mutexes, but it can of course generate mutex
                       groups which lead to *some* redundant mutexes,
                       where some but not all facts talk about the
                       same variable. */
                    g_inconsistent_facts[var1][val1].insert(fact2);
                }
            }
        }
    }
}

void update_initial_state(const GlobalState &newstate)
{
    if (g_state_registry) {
        delete(g_state_registry);
    }
    for (size_t var = 0; var < g_initial_state_data.size(); var++) {
        g_initial_state_data[var] = newstate[var];
    }
    g_state_registry = new StateRegistry(g_initial_state_data);
}

void read_goal(istream &in)
{
    check_magic(in, "begin_goal");
    int count;
    in >> count;
    if (count < 1) {
        cerr << "Task has no goal condition!" << endl;
        exit_with(EXIT_INPUT_ERROR);
    }
    for (int i = 0; i < count; ++i) {
        int var, val;
        in >> var >> val;
        g_goal.push_back(make_pair(var, val));
    }
    check_magic(in, "end_goal");
}

void dump_goal()
{
    cout << "Goal Conditions:" << endl;
    for (size_t i = 0; i < g_goal.size(); ++i)
        cout << "  " << g_variable_name[g_goal[i].first] << ": "
             << g_goal[i].second << endl;
}

void read_operators(istream &in)
{
    int count;
    in >> count;
    for (int i = 0; i < count; ++i) {
        g_operators.push_back(GlobalOperator(g_operators.size(), in, false));
    }
}

void read_axioms(istream &in)
{
    int count;
    in >> count;
    for (int i = 0; i < count; ++i) {
        g_axioms.push_back(GlobalOperator(g_operators.size(), in, true));
    }

    g_axiom_evaluator = new AxiomEvaluator;
}

void read_everything(istream &in)
{
    cout << "reading input... [t=" << g_timer << "]" << endl;
    read_and_verify_version(in);
    read_metric(in);
    read_variables(in);
    read_mutexes(in);
    g_initial_state_data.resize(g_variable_domain.size());
    check_magic(in, "begin_state");
    for (size_t i = 0; i < g_variable_domain.size(); ++i) {
        in >> g_initial_state_data[i];
    }
    check_magic(in, "end_state");
    g_default_axiom_values = g_initial_state_data;

    read_goal(in);
    read_operators(in);
    read_axioms(in);

    // Ignore successor generator from preprocessor output.
    check_magic(in, "begin_SG");
    while (!peek_magic(in, "end_SG")) {
        string dummy_string;
        getline(in, dummy_string);
    }
    check_magic(in, "end_SG");

    DomainTransitionGraph::read_all(in);
    check_magic(in, "begin_CG"); // ignore everything from here

    cout << "done reading input! [t=" << g_timer << "]" << endl;

    cout << "packing state variables..." << flush;
    assert(!g_variable_domain.empty());
    g_state_packer = new IntPacker(g_variable_domain);
    cout << "done! [t=" << g_timer << "]" << endl;

    // NOTE: state registry stores the sizes of the state, so must be
    // built after the problem has been read in.
    g_state_registry = new StateRegistry(g_initial_state_data);

    int num_vars = g_variable_domain.size();
    int num_facts = 0;
    for (int var = 0; var < num_vars; ++var) {
        num_facts += g_variable_domain[var];
    }

    cout << "Variables: " << num_vars << endl;
    cout << "Facts: " << num_facts << endl;
    cout << "Bytes per state: "
         << g_state_packer->get_num_bins() *
         g_state_packer->get_bin_size_in_bytes() << endl;

    cout << "Building successor generator..." << flush;
    g_successor_generator = new SuccessorGenerator(g_root_task());
    cout << "done! [t=" << g_timer << "]" << endl;

    cout << "Building causal graph ..." << endl;
    g_legacy_causal_graph = new LegacyCausalGraph();

    //cout << "Building probabilistic parts ..." << endl;
    if (has_axioms()) {
        cerr << "Axioms are currently not supported!" << endl;
        exit_with(EXIT_UNSUPPORTED);
    }
    cout << "Merging operator outcomes to probabilistic operators ..." << endl;
    map<string, int> name_to_i;
    vector<float> weights;
    vector<vector<pair<pair<float, float>, int> > > outcomes;
    int cur_i = 0;

    for (size_t i = 0; i < g_operators.size(); i++) {
        pair<pair<float, float>, string> data = parse_operator_outcome_name(
                g_operators[i].get_name());
        int j;
        map<string, int>::iterator it = name_to_i.find(data.second);
        if (it == name_to_i.end()) {
            j = cur_i;
            name_to_i[data.second] = cur_i++;
            g_prob_operators.push_back(vector<pair<float, int> >());
            g_prob_operator_name.push_back(data.second);
            weights.push_back(0);
            g_prob_operator_cost.push_back(g_operators[i].get_cost());
            outcomes.push_back(vector<pair<pair<float, float>, int> >());
        } else {
            j = it->second;
        }
        g_outcome_to_action.push_back(make_pair(j, outcomes[j].size()));
        outcomes[j].push_back(make_pair(data.first, i));
    }

    cout << "Computing probabilities ..." << endl;
    int tmp;
    for (size_t i = 0; i < outcomes.size(); i++) {
        int total = 1;
        int sum = 0;
        for (size_t j = 0; j < outcomes[i].size(); j++) {
            total = lcm(total, (int) outcomes[i][j].first.second);
        }
        for (size_t j = 0; j < outcomes[i].size(); j++) {
            tmp = total / outcomes[i][j].first.second;
            outcomes[i][j].first.first = outcomes[i][j].first.first * tmp;
            sum += outcomes[i][j].first.first;
            g_prob_operators[i].push_back(make_pair((float) outcomes[i][j].first.first /
                                                    total, outcomes[i][j].second));
        }
        if (sum < total) {
            //cout << "Probabilities do not sum up to one!" << endl;
            size_t dummy = g_operators.size();
            g_operators.push_back(GlobalOperator(g_operators.size(),
                                                 g_prob_operator_cost[i]));
            g_outcome_to_action.push_back(make_pair(i, g_prob_operators[i].size()));
            g_prob_operators[i].push_back(make_pair((float)(total - sum) / total, dummy));
        }
    }

    /*for (size_t i = 0; i < g_operators.size(); i++) {
        std::pair<int, int> out = g_outcome_to_action[g_operators[i].get_id()];
        if (g_prob_operators[out.first][out.second].second != g_operators[i].get_id())
            cout << "asdfabhe45t35345r::::::" << g_operators[i].get_id() << "::::" << i << endl;
    }*/

    cout << "Generating successor generator for probabilistic operators..." << endl;
    g_prob_successor_generator = new SuccessorGenerator(g_prob_task());

    //for (size_t i = 0; i < g_prob_operators.size(); i++) {
    //    g_prob_outcome_sampler.push_back(prob_outcome_sampler(g_prob_operators[i]));
    //}


    //cout << g_prob_operators.size() << " probabilistic operators " << endl;
    //for (size_t i = 0; i < g_prob_operators.size(); i++) {
    //    cout << "Operator #" << i << " with name " << g_prob_operator_name[i] << " has "
    //        << g_prob_operators[i].size() << " action outcomes: " << endl;
    //    for (size_t j = 0; j < g_prob_operators[i].size(); j++) {
    //        cout << " - outcome #" << j << " (" << g_prob_operators[i][j].first << ", "
    //            << g_prob_operators[i][j].second << "): "
    //         << "c = " << g_operators[g_prob_operators[i][j].second].get_cost() <<
    //         "; ";
    //        const GlobalOperator &op = g_operators[g_prob_operators[i][j].second];
    //        cout << "pre = {";
    //        bool sep = false;
    //        for (size_t i = 0; i < op.get_preconditions().size(); i++) {
    //            if (sep) {
    //                cout << ", ";
    //            }
    //            const GlobalCondition &cond = op.get_preconditions()[i];
    //            cout << g_fact_names[cond.var][cond.val];
    //            sep = true;
    //        }
    //        cout << "}; eff = {";
    //        sep = false;
    //        for (size_t i = 0; i < op.get_effects().size(); i++) {
    //            if (sep) {
    //                cout << ", ";
    //            }
    //            const GlobalEffect &cond = op.get_effects()[i];
    //            cout << g_fact_names[cond.var][cond.val];
    //            sep = true;
    //        }
    //        cout << "}" << endl;
    //    }
    //}


    cout << "done initalizing global data [t=" << g_timer << "]" << endl;
}

void dump_everything()
{
    cout << "Use metric? " << g_use_metric << endl;
    cout << "Min Action Cost: " << g_min_action_cost << endl;
    cout << "Max Action Cost: " << g_max_action_cost << endl;
    // TODO: Dump the actual fact names.
    cout << "Variables (" << g_variable_name.size() << "):" << endl;
    for (size_t i = 0; i < g_variable_name.size(); ++i)
        cout << "  " << g_variable_name[i]
             << " (range " << g_variable_domain[i] << ")" << endl;
    GlobalState initial_state = g_initial_state();
    cout << "Initial State (PDDL):" << endl;
    initial_state.dump_pddl();
    cout << "Initial State (FDR):" << endl;
    initial_state.dump_fdr();
    dump_goal();
    /*
    for(int i = 0; i < g_variable_domain.size(); ++i)
      g_transition_graphs[i]->dump();
    */
}

bool is_unit_cost()
{
    return g_min_action_cost == 1 && g_max_action_cost == 1;
}

bool has_axioms()
{
    return !g_axioms.empty();
}

void verify_no_axioms()
{
    if (has_axioms()) {
        cerr << "Heuristic does not support axioms!" << endl << "Terminating."
             << endl;
        exit_with(EXIT_UNSUPPORTED);
    }
}

static int get_first_conditional_effects_op_id()
{
    for (size_t i = 0; i < g_operators.size(); ++i) {
        const vector<GlobalEffect> &effects = g_operators[i].get_effects();
        for (size_t j = 0; j < effects.size(); ++j) {
            const vector<GlobalCondition> &cond = effects[j].conditions;
            if (!cond.empty()) {
                return i;
            }
        }
    }
    return -1;
}

bool has_conditional_effects()
{
    return get_first_conditional_effects_op_id() != -1;
}

void verify_no_conditional_effects()
{
    int op_id = get_first_conditional_effects_op_id();
    if (op_id != -1) {
        cerr << "Heuristic does not support conditional effects "
             << "(operator " << g_operators[op_id].get_name() << ")" << endl
             << "Terminating." << endl;
        exit_with(EXIT_UNSUPPORTED);
    }
}

void verify_no_axioms_no_conditional_effects()
{
    verify_no_axioms();
    verify_no_conditional_effects();
}

bool are_mutex(const pair<int, int> &a, const pair<int, int> &b)
{
    if (a.first == b.first) { // same variable: mutex iff different value
        return a.second != b.second;
    }
    return bool(g_inconsistent_facts[a.first][a.second].count(b));
}

const GlobalState &g_initial_state()
{
    return g_state_registry->get_initial_state();
}

const shared_ptr<AbstractTask> g_root_task()
{
    static shared_ptr<AbstractTask> root_task = make_shared<RootTask>();
    return root_task;
}

const shared_ptr<AbstractTask> g_prob_task()
{
    static shared_ptr<AbstractTask> prob_task = make_shared<ProbRootTask>();
    return prob_task;
}

//prob_outcome_sampler::prob_outcome_sampler(const std::vector<std::pair<float, int> > &outcomes)
//{
//    intervals.resize(outcomes.size());
//    if (intervals.size() > 0) {
//        intervals[0].first = 0;
//        intervals[0].second = outcomes[0].first;
//        for (size_t i = 1; i < outcomes.size(); i++) {
//            intervals[i].first = i;
//            intervals[i].second = intervals[i - 1].second + outcomes[i].first;
//        }
//    }
//}
//
//unsigned prob_outcome_sampler::operator()(float p)
//{
//    pair<unsigned, float> x = make_pair(0, p);
//    return std::lower_bound(intervals.begin(), intervals.end(), x, comp())->first;
//}


bool g_use_metric;
float g_min_action_cost = numeric_limits<float>::max();
float g_max_action_cost = 0;
vector<string> g_variable_name;
vector<int> g_variable_domain;
vector<vector<string> > g_fact_names;
vector<int> g_axiom_layers;
vector<int> g_default_axiom_values;
IntPacker *g_state_packer;
vector<int> g_initial_state_data;
vector<pair<int, int> > g_goal;
vector<GlobalOperator> g_operators;
vector<GlobalOperator> g_axioms;
AxiomEvaluator *g_axiom_evaluator;
SuccessorGenerator *g_successor_generator;
vector<DomainTransitionGraph *> g_transition_graphs;
LegacyCausalGraph *g_legacy_causal_graph;

// Marcel: stuff for supporting probabilities
vector<vector<pair<float, int> > > g_prob_operators;
vector<int> g_prob_operator_cost;
vector<string> g_prob_operator_name;
SuccessorGenerator *g_prob_successor_generator;
vector<pair<int, int> > g_outcome_to_action;
//vector<prob_outcome_sampler> g_prob_outcome_sampler;

Timer g_timer;
string g_plan_filename = "sas_plan";
int g_num_previously_generated_plans = 0;
bool g_is_part_of_anytime_portfolio = false;
RandomNumberGenerator g_rng(2011); // Use an arbitrary default seed.
StateRegistry *g_state_registry = 0;

