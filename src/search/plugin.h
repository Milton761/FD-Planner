#ifndef PLUGIN_H
#define PLUGIN_H

#include "open_lists/alternation_open_list.h"
#include "open_lists/standard_scalar_open_list.h"
#include "open_lists/tiebreaking_open_list.h"

#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

template <class T>
class Plugin
{
public:
    std::string key;
    Plugin(const std::string &key, typename Registry<T *>::Factory factory)
     : key(key) {
        Registry<T *>::instance()->register_object(key, factory);
    }
    ~Plugin() = default;
    Plugin(const Plugin<T> &other) = delete;
    const std::string &get_key() {
        return key;
    }
};


// TODO: This class will replace Plugin once we no longer need to support raw pointers.
template <class T>
class PluginShared
{
public:
    PluginShared(const std::string &key,
                 typename Registry<std::shared_ptr<T> >::Factory factory) {
        Registry<std::shared_ptr<T> >::instance()->register_object(key, factory);
    }
    ~PluginShared() = default;
    PluginShared(const PluginShared<T> &other) = delete;
};

template <class Entry>
class Plugin<OpenList<Entry > >
{
    Plugin(const Plugin<OpenList<Entry > > &copy);
public:
    ~Plugin();

    static void register_open_lists() {
        Registry<OpenList<Entry > *>::instance()->register_object(
            "single", StandardScalarOpenList<Entry>::_parse);
        Registry<OpenList<Entry > *>::instance()->register_object(
            "tiebreaking", TieBreakingOpenList<Entry>::_parse);
        Registry<OpenList<Entry > *>::instance()->register_object(
            "alt", AlternationOpenList<Entry>::_parse);
    }
};

//template<class SearchNode, class SearchSpace>
//class Plugin<PolicySimulator<SearchNode, SearchSpace> >
//{
//    Plugin(const Plugin<PolicySimulator<SearchNode, SearchSpace> > &copy);
//public:
//    ~Plugin();
//
//    static void register_policy_simulators() {
//        Registry<PolicySimulator<SearchNode, SearchSpace> *>::instance()->register_object(
//                "avg", PolicySimulator<SearchNode, SearchSpace>::_parse);
//    }
//};

#endif
