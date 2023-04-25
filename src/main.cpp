#include <iostream>
#include <signal.h>
#include <memory>

#include "my_VO.h"
#include "logger.hpp"

const std::string config_file("../config/vo_config.json");

std::unique_ptr<MyVO> vo;

void signal_callback_handler(int signum) {
    logger::LogInfo("main", "Ctrl+c detected!");
    vo->Stop();
}

int main() {
    signal(SIGINT, signal_callback_handler);
    signal(SIGTERM, signal_callback_handler);

    try {
        vo = std::make_unique<MyVO>(config_file);
        vo->Run();
    } catch(const std::invalid_argument& e) {
        logger::LogError("main", e.what());
    }

    logger::LogInfo("main", "VO exited successfuly...");

    return 0;
}