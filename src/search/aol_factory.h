
#ifndef AOL_FACTORY_H
#define AOL_FACTORY_H

#include "prob_search_engine_factory.h"

#define COMPILE_AOL_TIPLIST_FIFO 1
#define COMPILE_AOL_TIPLIST_LIFO 1
#define COMPILE_AOL_TIPLIST_DASC 0
#define COMPILE_AOL_TIPLIST_DDESC 0
#define COMPILE_AOL_TIPLIST_HASC 0
#define COMPILE_AOL_TIPLIST_HDESC 0
#define COMPILE_AOL_TIPLIST_DD 0
#define COMPILE_AOL_TIPLIST_BASC 0
#define COMPILE_AOL_TIPLIST_BESC 0
#define COMPILE_AOL_TIPLIST_DDd 0

namespace prob_search_engine_factory {
    extern Plugin<SearchEngine> aol;
}


#endif
