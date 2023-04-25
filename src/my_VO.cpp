#include "my_VO.h"

#include <map>

MyVO::MyVO(const std::string &conf_file):
            running_(true) {
    ConfigureVO(conf_file);
    
    std::vector<std::string> temp_image_path_buff;
    switch (mode_) {
    case IMAGES:
        for(auto & p : boost::filesystem::directory_iterator(dataset_path_+"frames/")) {
            temp_image_path_buff.push_back(p.path().c_str());
        }
        std::sort(temp_image_path_buff.begin(), temp_image_path_buff.end());
        for (const auto &data : temp_image_path_buff) {
            frames_.push(data);
        }

        break;
    case VIDEO:
        cap_ = std::make_unique<cv::VideoCapture>(dataset_path_ + "video/vo_video.mp4");

        if (!cap_->isOpened()) {
            throw std::invalid_argument("Error opening video stream or file!");
        }

        break;
    }
}

MyVO::~MyVO() {
    if (cap_ != nullptr) {
        cap_->release();
    }

    cv::destroyAllWindows();
}

void MyVO::Run() {
    while (running_.load()) {
        cv::Mat frame;

        frame = ReadCurrentFrame();

        if (frame.empty())  break;

        if (!is_VO_initilaized_) {
            current_frame_ = frame.clone();

            // initilaize VO

            is_VO_initilaized_ = true;
            continue;
        }

        fm_.Match(current_frame_, frame);
        current_frame_ = frame.clone();

        cv::Mat img_match = fm_.DrawMatches();

        imshow( "Frame with matches", img_match );

        char c = (char)cv::waitKey(25);
    }
}

void MyVO::InitializeVO() {

}

void MyVO::ConfigureVO(const std::string& conf_file) {
    Config::LoadConfigFile(conf_file);

    json config = Config::GetConfigData();

    std::string dataset_name = config["dataset_name"].get<std::string>();

    dataset_path_ = config[dataset_name]["dataset_dir"].get<std::string>();
    mode_ = static_cast<MODE>(config[dataset_name]["mode"].get<int>());
    num_images_ = config[dataset_name]["num_images"].get<size_t>();
    double fx = config[dataset_name]["camera_info"]["fx"].get<double>();
    double fy = config[dataset_name]["camera_info"]["fy"].get<double>();
    double cx = config[dataset_name]["camera_info"]["cx"].get<double>();
    double cy = config[dataset_name]["camera_info"]["cy"].get<double>();
    K_ = (cv::Mat_<double>(3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
}

cv::Mat MyVO::ReadCurrentFrame() {
    cv::Mat frame;

    switch (mode_) {
    case IMAGES:
        if (frames_.empty())    break;
        frame = cv::imread(frames_.front(), cv::IMREAD_COLOR);
        frames_.pop();
        break;
    case VIDEO:
        *cap_ >> frame;
        break;
    default:
        break;
    }

    return frame;
}

void MyVO::Stop() {
    running_.store(false);
}