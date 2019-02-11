
#include "tarjan_dfs_factory.h"

#include "string_ops.h"

namespace tarjan_dfs_factory
{
void add_options_to_parser(OptionParser &parser)
{
    prob_search_engine::add_options_to_parser(parser);

    std::vector<std::string> tiebreaking;
    action_tie_breaking::get_options(tiebreaking);
    parser.add_enum_option("tiebreaking", tiebreaking, "", tiebreaking.front());

    parser.add_option<float>("epsilon", "", "0");

    parser.add_option<bool>("labeling", "", "false");
    parser.add_option<bool>("fw_updates", "", "true");
    parser.add_option<bool>("bw_updates", "", "false");
    parser.add_option<bool>("inconsistent", "", "false");
    parser.add_option<bool>("terminate_trial", "", "false");

    parser.add_option<bool>("vi", "", "true");

    parser.add_option<bool>("cache_flags", "", "false");
}

void copy_options_to_parser(const Options &opts, OptionParser &parser)
{
    std::vector<std::string> tiebreaking;
    action_tie_breaking::get_options(tiebreaking);
    parser.add_enum_option("tiebreaking",
                           tiebreaking,
                           "",
                           tiebreaking[opts.get_enum("tiebreaking")]);
    parser.add_option<bool>("labeling", "",
                            string_ops::to_string(opts.get<bool>("labeling")));
    parser.add_option<bool>("fw_updates", "",
                            string_ops::to_string(opts.get<bool>("fw_updates")));
    parser.add_option<bool>("bw_updates", "",
                            string_ops::to_string(opts.get<bool>("bw_updates")));
    parser.add_option<bool>("inconsistent", "",
                            string_ops::to_string(opts.get<bool>("inconsistent")));
    parser.add_option<bool>("terminate_trial", "",
                            string_ops::to_string(opts.get<bool>("terminate_trial")));
    parser.add_option<bool>("vi", "",
                            string_ops::to_string(opts.get<bool>("vi")));
    parser.add_option<bool>("cache_flags", "",
                            string_ops::to_string(opts.get<bool>("cache_flags")));
}

void consistency_check(const Options &opts)
{
    if (!opts.get<bool>("vi") && !opts.get<bool>("labeling")) {
        std::cerr << "you have to set either vi or labeling" << std::endl;
        exit_with(EXIT_CRITICAL_ERROR);
    }
    if (!opts.get<bool>("bw_updates") && !opts.get<bool>("vi")) {
        std::cerr << "you have to set at least one of vi and bw_update" << std::endl;
        exit_with(EXIT_CRITICAL_ERROR);
    }
    if (opts.get<bool>("inconsistent") && !opts.get<bool>("fw_updates")) {
        std::cerr << "inconsistencies can be only checked by doing forward updates" <<
                  std::endl;
        exit_with(EXIT_CRITICAL_ERROR);
    }
    if (opts.get<bool>("labeling") && !opts.get<bool>("fw_updates")) {
        std::cerr <<
                  "in order to label states solved, we have to check for inconsistent states which requires forward updates"
                  << std::endl;
        exit_with(EXIT_CRITICAL_ERROR);
    }
}
}

namespace prob_search_engine_factory
{
Plugin<SearchEngine> hdfs("hdfs", tarjan_dfs_factory::parse<ProbabilisticSearchEngine>);
}



