#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include <string>
#include <sstream>

#include <yaml-cpp/yaml.h>

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
    
    template <class T>
    static T GetParam(const std::string &param_name);

    Config(const Config&) = delete;
    Config(Config&&) = delete;
    Config& operator=(const Config&) = delete;
    Config& operator=(Config&&) = delete;

private:
    static YAML::Node config_;
};

template <class T>
T Config::GetParam(const std::string &param_name) {
    std::stringstream ss(param_name);
    std::string param;

    bool first_flag = true;

    YAML::Node temp_node;

    while (getline(ss, param, '/')) {
        if (first_flag) {
            first_flag = false;
            temp_node = config_[param];
            continue;
        }
        temp_node = temp_node[param];
    }

    return temp_node.as<T>();
}

#endif // CONFIG_HPP_