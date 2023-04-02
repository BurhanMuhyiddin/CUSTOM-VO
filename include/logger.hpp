#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <iostream>
#include <string>

namespace logger {
    inline void LogInfo(const std::string &name, const std::string &msg) {
        std::cout << "[" << name << "] " << msg << "\n";
    }

    inline void LogError(const std::string &name, const std::string &msg) {
        std::cerr << "[" << name << "] " << msg << "\n";
    }
}

#endif // LOGGER_HPP_