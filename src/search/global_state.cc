#include "global_state.h"

#include "globals.h"
#include "utilities.h"
#include "state_registry.h"

#include <algorithm>
#include <iostream>
#include <cassert>
using namespace std;


GlobalState::GlobalState(const PackedStateBin *buffer_,
                         const StateRegistry &registry_,
                         StateID id_)
    : buffer(buffer_),
      registry(&registry_),
      id(id_)
{
    assert(buffer);
    assert(id != StateID::no_state);
}

GlobalState::~GlobalState()
{
}

int GlobalState::operator[](size_t index) const
{
    return g_state_packer->get(buffer, index);
}

void GlobalState::dump_pddl(std::ostream &out, bool newline) const
{
    if (newline) {
        for (size_t i = 0; i < g_variable_domain.size(); ++i) {
            const string &fact_name = g_fact_names[i][(*this)[i]];
            if (fact_name != "<none of those>") {
                out << fact_name << endl;
            }
        }
    } else {
        bool sep = false;
        for (size_t i = 0; i < g_variable_domain.size(); ++i) {
            const string &fact_name = g_fact_names[i][(*this)[i]];
            if (fact_name != "<none of those>") {
                if (sep) {
                    out << " ";
                }
                sep = true;
                out << fact_name;
            }
        }
    }
}

void GlobalState::dump_fdr() const
{
    for (size_t i = 0; i < g_variable_domain.size(); ++i)
        cout << "  #" << i << " [" << g_variable_name[i] << "] -> "
             << (*this)[i] << endl;
}
