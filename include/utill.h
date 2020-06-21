#ifndef COLUGO_UTILL_H_
#define COLUGO_UTILL_H_

#include <queue>
#include <memory>
#include <string>
using namespace std ;

#define DEBUG

#ifdef DEBUG
#define DEBUG_TEST 1
#else
#define DEBUG_TEST 0
#endif


#define debug_print(fmt, ...) \
            do { if (DEBUG_TEST) fprintf(stderr, fmt, ## __VA_ARGS__); } while (0)
                

#include <colugo_broker.h>
using namespace ColugoBrokerModule ;

#endif
