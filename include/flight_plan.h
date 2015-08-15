#ifndef FLIGHT_PLAN_HEADER
#define FLIGHT_PLAN_HEADER

#include <string>
#include <iostream>

#define MAX_BLOCK_COUNT 20
#define MAX_FUNCTION_COUNT 100
#define MAX_FUNCTION_ARG_COUNT 5

enum function_type { khar = 1,
                     sag,
                     gaav

};

struct function {
    function_type type;
    float arg[MAX_FUNCTION_ARG_COUNT];
    bool done = false;
};

struct block {
    unsigned short int start_function_number;
    std::string name;
};


#endif
