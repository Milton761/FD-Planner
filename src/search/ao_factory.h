
#ifndef AO_FACTORY_H
#define AO_FACTORY_H

#include "prob_search_engine_factory.h"

#define COMPILE_AO_OSELECT_MOST_LIKELY 0
#define COMPILE_AO_OSELECT_PREFERRED 0
#define COMPILE_AO_OSELECT_GAP 0
#define COMPILE_AO_OSELECT_MIN_H 0

namespace prob_search_engine_factory {
    extern Plugin<SearchEngine> ao;
}


#endif
