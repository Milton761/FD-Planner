
#ifndef TEMPLATE_FACTORY_H
#define TEMPLATE_FACTORY_H

#include "option_parser.h"

struct TemplateFactory
{
protected:
    Options opts;
public:
    virtual ~TemplateFactory() {}
    const Options &get_options() const {
        return opts;
    }
};

#endif
