
#include "action_tie_breaking.h"


namespace action_tie_breaking {
    void get_options(std::vector<std::string> &opts) {
        opts.push_back("arbitrary");
        opts.push_back("minh");
        opts.push_back("mingap");
        opts.push_back("maxgap");
        opts.push_back("preferred");
    }
}

