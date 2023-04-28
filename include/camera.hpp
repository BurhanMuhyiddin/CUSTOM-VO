#ifndef CAMERA_HPP_
#define CAMERA_HPP_

#include "opencv2/opencv.hpp"

cv::Point2f pixel_to_cam_norm_plane(const cv::Point2f &p, const cv::Mat &K) {
    return cv::Point2f(
        (p.x - K.at<float>(0, 2)) / K.at<float>(0, 0),
        (p.y - K.at<float>(1, 2)) / K.at<float>(1, 1)
    );
}

cv::Point3f world_to_pixel(const cv::Point3f &p, const cv::Mat &T, bool &is_in_front) {
    is_in_front = false;
    cv::Mat p_homogenous = (cv::Mat_<float>(4,1) << p.x, p.y, p.z, 1.0);

    cv::Mat pixel_point = T * p_homogenous;

    if (pixel_point.at<float>(2, 0) > 0)
        is_in_front = true;

    pixel_point /= pixel_point.at<float>(2, 0);

    return cv::Point3f (
        pixel_point.at<float>(0,0),
        pixel_point.at<float>(1,0),
        pixel_point.at<float>(2,0)
    );
}

#endif // CAMERA_HPP_