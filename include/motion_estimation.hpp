#ifndef MOTION_ESTIMATION_HPP_
#define MOTION_ESTIMATION_HPP_

#include <vector>

#include "opencv2/opencv.hpp"

#include "camera.hpp"

void filter_homography_decomposition(const std::vector<cv::Mat> &Rs,
                                    const std::vector<cv::Mat> &ts,
                                    const std::vector<cv::Mat> &ns,
                                    const std::vector<cv::Point2f> &points1,
                                    const std::vector<cv::Point2f> &points2,
                                    cv::Mat &R,
                                    cv::Mat &t,
                                    const std::vector<int> &inliers_index) {
    cv::Mat possible_solutions;
    cv::filterHomographyDecompByVisibleRefpoints(Rs, ns, points1, points2, possible_solutions, inliers_index);

    std::vector<cv::Mat> filtered_Rs, filtered_ts, filtered_ns;
    for (int i = 0; i < possible_solutions.rows; ++i) {
        int indx = possible_solutions.at<int>(i, 0);
        filtered_Rs.push_back(Rs[indx]);
        filtered_ts.push_back(ts[indx]);
        filtered_ns.push_back(ns[indx]);
    }

    int best_solution_indx = 0;
    double max_norm_z = 0.0;
    for (int i = 0; i < filtered_ns.size(); ++i) {
        double norm_z = fabs(filtered_ns[i].at<double>(2, 0));
        if (norm_z > max_norm_z) {
            max_norm_z = norm_z;
            best_solution_indx = i;
        }
    }

    R = filtered_Rs[best_solution_indx];
    t = filtered_ts[best_solution_indx];
}

void estimate_motion_from_homography_mat(cv::Mat &H,
            cv::Mat &K,
            const std::vector<cv::Point2f> &points1,
            const std::vector<cv::Point2f> &points2,
            cv::Mat &R,
            cv::Mat &t,
            const std::vector<int> &inliers_index) {
    std::vector<cv::Mat> Rs, ts, ns;
    cv::decomposeHomographyMat(H, K, Rs, ts, ns);

    // filter for 1 R and t
    std::vector<cv::Point2f> cam_points1;
    std::vector<cv::Point2f> cam_points2;

    for (int i = 0; i < points1.size(); ++i) {
        cam_points1.push_back(pixel_to_cam_norm_plane(points1[i], K));
        cam_points2.push_back(pixel_to_cam_norm_plane(points2[i], K));
    }

    filter_homography_decomposition(Rs, ts, ns, cam_points1, cam_points2, R, t, inliers_index);
}

void estimate_motion_from_essential_mat(cv::Mat &E,
                                        cv::Mat &K,
                                        const std::vector<cv::Point2f> &points1,
                                        const std::vector<cv::Point2f> &points2,
                                        cv::Mat &R,
                                        cv::Mat &t,
                                        const std::vector<int> &inliers_index) {
    
    cv::recoverPose(E, points1, points2, K, R, t, inliers_index);
}

#endif // MOTION_ESTIMATION_HPP_