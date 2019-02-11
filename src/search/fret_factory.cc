
#include "fret_factory.h"

#include "fret.h"

#include "prob_search_engine_factory.h"

#include "optimization_criteria.h"
#include "action_tie_breaking.h"

#include "option_parser.h"
#include "plugin.h"

#include "utilities.h"

#include <iostream>
#include <vector>
#include <string>

namespace fret_factory
{
static Plugin<FRETEngine> e_lrtdp("lrtdp", LRTDPFRETEngine::parse);
static Plugin<FRETEngine> e_hdfs("hdfs", DFSFRETEngine::parse);

void add_options_to_parser(OptionParser &parser, const std::string &alg)
{
    prob_search_engine::add_options_to_parser(parser);
    parser.add_option<float>("epsilon", "", "0.00005");
    parser.add_option<float>("delta", "", "0.0005");
    parser.add_option<bool>("local", "", "true");
    parser.add_option<FRETEngine *>("engine", "", alg);
}

SearchEngine *parse(OptionParser &parser)
{
    add_options_to_parser(parser, "lrtdp");
    Options opts = parser.parse();
    if (!parser.dry_run()) {
        FRETEngine *engine = opts.get<FRETEngine *>("engine");
        return engine->generate(parser);
    }
    return NULL;
}
}

Plugin<SearchEngine> prob_search_engine_factory::fret("fret",
        fret_factory::parse);

