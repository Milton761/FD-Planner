
#ifndef STATE_SPACE_LOADER_H
#define STATE_SPACE_LOADER_H

#include "template_factory.h"

#include "plugin.h"
#include "option_parser.h"

#include "state_space.h"
#include "mas_state_space.h"

namespace state_space_factory
{

enum Types {
    FACTORED = 0,
    MERGE_AND_SHRINK = 1,
};

struct StateSpaceFactory : public TemplateFactory {
    virtual Types get_type() const {
        return FACTORED;
    }
};

struct StateSpaceLookupFactory : public StateSpaceFactory {
public:
    StateSpaceLookupFactory(const Options &opts) {
        state_space_lookup::copy_options(opts, this->opts);
    }
    virtual Types get_type() const {
        return FACTORED;
    }
    static StateSpaceFactory *_parse(OptionParser &parser) {
        state_space_lookup::add_options_to_parser(parser);
        Options opts = parser.parse();
        if (!parser.dry_run()) {
            return new StateSpaceLookupFactory(opts);
        }
        return NULL;
    }
};

struct StateSpaceMASFactory : public StateSpaceFactory {
public:
    StateSpaceMASFactory(const Options &opts) {
        state_space_mas::copy_options(opts, this->opts);
    }
    virtual Types get_type() const {
        return MERGE_AND_SHRINK;
    }
    static StateSpaceFactory *_parse(OptionParser &parser) {
        state_space_mas::add_options_to_parser(parser);
        Options opts = parser.parse();
        if (!parser.dry_run()) {
            return new StateSpaceMASFactory(opts);
        }
        return NULL;
    }
};

static Plugin<StateSpaceFactory> _plugin_lookup("factored",
        StateSpaceLookupFactory::_parse);
static Plugin<StateSpaceFactory> _plugin_mas("mas",
        StateSpaceMASFactory::_parse);

}

#endif

