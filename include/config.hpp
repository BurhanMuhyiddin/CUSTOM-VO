#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include <string>
#include <fstream>
#include <sstream>

#include "json.hpp"

using json = nlohmann::json;

class Config
{
protected:
    Config() = default;

public:
    static Config& GetInstance() {
        static Config instance;
        return instance;
    }

    static void LoadConfigFile(const std::string &config_file);
    
    static json GetConfigData() { return config_; }

    Config(const Config&) = delete;
    Config(Config&&) = delete;
    Config& operator=(const Config&) = delete;
    Config& operator=(Config&&) = delete;

private:
    static json config_;
};

#endif // CONFIG_HPP_