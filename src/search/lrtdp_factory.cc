#include "lrtdp_factory.h"

#include "string_ops.h"

namespace lrtdp_factory
{
void add_options_to_parser(OptionParser &parser)
{
    prob_search_engine::add_options_to_parser(parser);
    parser.add_option<float>("epsilon", "", "0");
    parser.add_option<int>("seed", "", "99999");
    parser.add_option<bool>("epsconsist", "", "false");

    std::vector<std::string> oselect;
    oselect.push_back("default");
    oselect.push_back("gap");
    oselect.push_back("h");
    parser.add_enum_option("oselect", oselect, "", "default");

    std::vector<std::string> tiebreaking;
    action_tie_breaking::get_options(tiebreaking);
    parser.add_enum_option("tiebreaking", tiebreaking, "", tiebreaking.front());
}

void copy_options_to_parser(const Options &opts, OptionParser &parser)
{
    parser.add_option<float>("epsilon", "",
                             string_ops::to_string(opts.get<float>("epsilon")));
    parser.add_option<int>("seed", "",
                           string_ops::to_string(opts.get<int>("seed")));
    parser.add_option<bool>("epsconsist", "",
                            string_ops::to_string(opts.get<bool>("epsconsist")));

    std::vector<std::string> oselect;
    oselect.push_back("default");
    oselect.push_back("gap");
    oselect.push_back("h");
    parser.add_enum_option("oselect", oselect, "",
                           string_ops::to_string(opts.get_enum("oselect")));

    std::vector<std::string> tiebreaking;
    action_tie_breaking::get_options(tiebreaking);
    parser.add_enum_option("tiebreaking", tiebreaking, "",
                           string_ops::to_string(opts.get_enum("tiebreaking")));
}
}

namespace prob_search_engine_factory
{
Plugin<SearchEngine> lrtdp("lrtdp", lrtdp_factory::parse<ProbabilisticSearchEngine>);
}


