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

bool triangulate_two_frames(const std::vector<cv::Point2f> &points1,
                            const std::vector<cv::Point2f> &points2,
                            const cv::Mat &T1,
                            const cv::Mat &T2,
                            const cv::Mat &K,
                            std::vector<cv::Point3f> &xyz_points,
                            double min_parallax) {
    //    cv::triangulatePoints()
    std::vector<cv::Point2f> cam_points1, cam_points2;
    for (int i = 0; i < points1.size(); ++i) {
        cam_points1.push_back(pixel_to_cam_norm_plane(points1[i], K));
        cam_points2.push_back(pixel_to_cam_norm_plane(points2[i], K));
    }

    cv::Mat pts_4d;
    cv::triangulatePoints(T1, T2, cam_points1, cam_points2, pts_4d);

    for (int i = 0; i < pts_4d.cols; ++i) {
        cv::Mat x = pts_4d.col(i);
        x /= x.at<float>(3, 0);
        xyz_points.push_back(cv::Point3f(x.at<float>(0,0),
                                         x.at<float>(1,0),
                                         x.at<float>(2,0)));
    }

    // check for validity of triangulation via checking parallax
    cv::Mat xyz_points_mat = cv::Mat(xyz_points).reshape(1);
    cv::Mat translation1 = T1.col(T1.cols-1).t();
    cv::Mat translation2 = T2.col(T2.cols-1).t();
    cv::Mat rays1 = xyz_points_mat.clone();
    cv::Mat rays2 = xyz_points_mat.clone();

    for (int i = 0; i < xyz_points_mat.rows; ++i) {
        rays1.row(i) = xyz_points_mat.row(i) - translation1;
        rays2.row(i) = xyz_points_mat.row(i) - translation2;
    }

    cv::Mat numerator, denominator;
    cv::reduce(rays1.mul(rays2), numerator, 1, cv::REDUCE_SUM);
    cv::Mat rays1_norm = cv::Mat::zeros(rays1.rows, 1, CV_32F);
    cv::Mat rays2_norm = cv::Mat::zeros(rays2.rows, 1, CV_32F);;
    for (int i = 0; i < rays1.rows; ++i) {
        rays1_norm.at<float>(i, 0) = cv::norm(rays1.row(i), cv::NORM_L2);
        rays2_norm.at<float>(i, 0) = cv::norm(rays2.row(i),cv::NORM_L2);
    }
    denominator = rays1_norm.mul(rays2_norm);
    cv::Mat cos_angles = numerator / denominator;
    double min_angle, max_angle;
    cv::minMaxLoc(cos_angles, &min_angle, &max_angle);
    if (min_angle > 0 && max_angle < cos(min_parallax))
        return true;
    return false;
}

#endif // MOTION_ESTIMATION_HPP_