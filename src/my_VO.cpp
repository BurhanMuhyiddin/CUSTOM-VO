#include "my_VO.h"

#include <map>

MyVO::MyVO(const std::string &conf_file):
            running_(true) {
    ConfigureVO();
    
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

void MyVO::ConfigureVO() {
    std::string dataset_name = Config::GetInstance().GetParam<std::string>("dataset_name");

    dataset_path_ = Config::GetInstance().GetParam<std::string>(dataset_name+"/dataset_dir");
    mode_ = static_cast<MODE>(Config::GetInstance().GetParam<int>(dataset_name+"/mode"));
    num_images_ = Config::GetInstance().GetParam<size_t>(dataset_name+"/num_images");
    auto camera_info = Config::GetInstance().GetParam<std::map<std::string, double>>(dataset_name+"/camera_info");
    double fx = camera_info["fx"];
    double fy = camera_info["fy"];
    double cx = camera_info["cx"];
    double cy = camera_info["cy"];
    K_ = (cv::Mat_<double>(3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
}

cv::Mat MyVO::ReadCurrentFrame() {
    cv::Mat frame;

    switch (mode_) {
    case IMAGES:
        if (frames_.empty())    break;
        frame = cv::imread(frames_.front(), CV_LOAD_IMAGE_COLOR);
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