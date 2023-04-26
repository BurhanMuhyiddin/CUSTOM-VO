#ifndef MY_VO_H_
#define MY_VO_H_

#include <string>
#include <atomic>
#include <memory>
#include <queue>
#include <vector>
#include <algorithm>

#include "opencv2/opencv.hpp"

#include <boost/filesystem.hpp>

#include "logger.hpp"
#include "feature_ORB.hpp"
#include "config.hpp"

namespace fs = boost::filesystem;

enum MODE { IMAGES = 0, VIDEO };

class MyVO {
public:
    MyVO(const std::string &conf_file);

    ~MyVO();

    void Run();

    bool InitializeVO();

    void Stop();
private:
    void ConfigureVO(const std::string& conf_file);
    cv::Mat ReadCurrentFrame();
private:
    bool is_VO_initilaized_ = false;
    std::unique_ptr<cv::VideoCapture> cap_;

    std::atomic<bool> running_;

    cv::Mat current_frame_;

    FeatureMatcher fm_;

    MODE mode_;
    size_t num_images_;
    std::string dataset_path_;
    cv::Mat K_; // intrinsic camera matrix

    std::queue<std::string> frames_;
};

#endif // MY_VO_H_