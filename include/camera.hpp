#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include "opencv2/opencv.hpp"

cv::Point2f pixel_to_cam_norm_plane(const cv::Point2f &p, const cv::Mat &K) {
    return cv::Point2f(
        (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
        (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
    );
}

#endif // CAMERA_HPP_