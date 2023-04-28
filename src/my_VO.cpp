#include <map>
#include <stdexcept>

#include "my_VO.h"
#include "epipolar_geometry.hpp"
#include "motion_estimation.hpp"
#include "helper_functions.hpp"

#define PI 3.14159265359
#define DEG2RAD(d)  (d * PI / 180.0)

cv::Mat rigid_transform = (cv::Mat_<float>(3,4) << 1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0);
double min_parallax = 1;

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
    EpipolarGeometry::SetIntrinsicCameraMatrix(K_);

    if (!InitializeVO()) {
        throw std::runtime_error("Map cannot be initialized!\n");
    }

    while (running_.load()) {
        cv::Mat frame;

        frame = ReadCurrentFrame();

        if (frame.empty())  break;

        fm_.Match(current_frame_, frame);
        current_frame_ = frame.clone();

        cv::Mat img_match = fm_.DrawMatches();

        imshow( "Frame with matches", img_match );

        char c = (char)cv::waitKey(25);
    }
}

bool MyVO::InitializeVO() {
    bool is_map_initialized = false;
    bool is_homography = false;

    cv::Mat F, H, E, transformMat;
    double Fscore, Hscore;
    cv::Mat Finliers_mask, Hinliers_mask, inliers_mask;

    size_t min_num_matches = 100;
    double ratio_threshold = 0.45;

    auto first_frame = ReadCurrentFrame();
    while (running_.load()) {
        auto frame = ReadCurrentFrame();

        fm_.Match(frame, first_frame);

        if (fm_.GetNumMatchedPoints() < min_num_matches)
            continue;

        auto matched_points = fm_.GetMatchedPoints();

        EpipolarGeometry::SetPoints(matched_points.first, matched_points.second);

        // calculate fundamental matrix with its score
        EpipolarGeometry::GetFundamentalMatrix(F, Fscore, Finliers_mask);

        // calculate essential matrix
        F.convertTo(F, CV_32F);
        E = K_.t() * F * K_;

        // calculate homography matrix with its score
        EpipolarGeometry::GetHomographyMatrix(H, Hscore, Hinliers_mask);

        // select model based on heuristic
        double ratio = Hscore / (Hscore + Fscore);
        if (ratio > ratio_threshold) {
            transformMat = H;
            inliers_mask = Hinliers_mask;

            is_homography = true;
        } else {
            transformMat = F;
            inliers_mask = Finliers_mask;

            is_homography = false;
        }

        double inlier_points_ratio = cv::countNonZero(inliers_mask)*1.0 / inliers_mask.rows*1.0;
        if (inlier_points_ratio < 0.90)
            continue;

        cv::Mat R, t;
        if (is_homography) {
            estimate_motion_from_homography_mat(H, K_, matched_points.first, matched_points.second, R, t, Hinliers_mask);
            if (R.empty()) {
                estimate_motion_from_essential_mat(E, K_, matched_points.first, matched_points.second, R, t, Finliers_mask);
            }
        } else {
            estimate_motion_from_essential_mat(E, K_, matched_points.first, matched_points.second, R, t, Finliers_mask);
        }

        // do triangulation
        std::vector<cv::Point3f> xyz_points;
        cv::Mat T2;
        helper::from_Rt_to_T(R, t, T2);
        std::vector<int> triangulation_inlier_indices;
        if (!triangulate_two_frames(matched_points.first,
                                    matched_points.second,
                                    rigid_transform,
                                    T2.rowRange(0, 3),
                                    K_,
                                    xyz_points,
                                    triangulation_inlier_indices,
                                    DEG2RAD(min_parallax)))     continue;
    }

    return is_map_initialized;
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
    K_ = (cv::Mat_<float>(3,3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
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