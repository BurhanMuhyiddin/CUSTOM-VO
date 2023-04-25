#include "config.hpp"

json Config::config_;

void Config::LoadConfigFile(const std::string &config_file) {
    std::ifstream f(config_file);
    config_ = json::parse(f);
}