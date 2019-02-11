
#ifndef ACYCLIC_DFS_FACTORY_H
#define ACYCLIC_DFS_FACTORY_H

#include "prob_search_engine_factory.h"

#define COMPILE_ACYCLIC_DFS 0

#if COMPILE_ACYCLIC_DFS
namespace prob_search_engine_factory {
    extern Plugin<SearchEngine> ahdfs;
}
#endif

#endif

