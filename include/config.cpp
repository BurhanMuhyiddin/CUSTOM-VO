#include "config.hpp"

YAML::Node Config::config_;

void Config::LoadConfigFile(const std::string &config_file) {
    config_ = YAML::LoadFile(config_file);
}