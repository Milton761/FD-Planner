
#include "string_ops.h"

#include <sstream>

namespace string_ops {
    std::string to_string(int x) {
        std::ostringstream ss;
        ss << x;
        return ss.str();
    }
    std::string to_string(float x) {
        std::ostringstream ss;
        ss << x;
        return ss.str();
    }
    std::string to_string(bool x) {
        std::ostringstream ss;
        ss << (x ? "true" : "false");
        return ss.str();
    }
    std::string to_string(unsigned x) {
        std::ostringstream ss;
        ss << x;
        return ss.str();
    }
}

