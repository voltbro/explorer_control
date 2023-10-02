#include <chrono>

#include "utils.h"

uint64_t timeMillis() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}
