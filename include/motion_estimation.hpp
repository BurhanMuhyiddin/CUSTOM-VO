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
                                    const cv::Mat &inliers_mask) {
    cv::Mat possible_solutions;
    cv::filterHomographyDecompByVisibleRefpoints(Rs, ns, points1, points2, possible_solutions, inliers_mask);
    std::vector<cv::Mat> filtered_Rs, filtered_ts, filtered_ns;
    if (possible_solutions.rows == 0)   return;

    for (int i = 0; i < possible_solutions.rows; ++i) {
        int index = possible_solutions.at<int>(i, 0);
        filtered_Rs.push_back(Rs[index]);
        filtered_ts.push_back(ts[index]);
        filtered_ns.push_back(ns[index]);
    }

    int best_solution_index = 0;
    double max_norm_z = 0.0;

    for (int i = 0; i < filtered_ns.size(); ++i) {
        double norm_z = fabs(filtered_ns[i].at<double>(2, 0));
        if (norm_z > max_norm_z) {
            max_norm_z = norm_z;
            best_solution_index = i;
        }
    }

    R = filtered_Rs[best_solution_index];
    t = filtered_ts[best_solution_index];
}

void estimate_motion_from_homography_mat(cv::Mat &H,
            cv::Mat &K,
            const std::vector<cv::Point2f> &points1,
            const std::vector<cv::Point2f> &points2,
            cv::Mat &R,
            cv::Mat &t,
            const cv::Mat &inliers_mask) {
    std::vector<cv::Mat> Rs, ts, ns;
    cv::decomposeHomographyMat(H, K, Rs, ts, ns);

    // filter for 1 R and t
    std::vector<cv::Point2f> cam_points1;
    std::vector<cv::Point2f> cam_points2;

    for (int i = 0; i < points1.size(); ++i) {
        cam_points1.push_back(pixel_to_cam_norm_plane(points1[i], K));
        cam_points2.push_back(pixel_to_cam_norm_plane(points2[i], K));
    }

    filter_homography_decomposition(Rs, ts, ns, cam_points1, cam_points2, R, t, inliers_mask);
}

void estimate_motion_from_essential_mat(cv::Mat &E,
                                        cv::Mat &K,
                                        const std::vector<cv::Point2f> &points1,
                                        const std::vector<cv::Point2f> &points2,
                                        cv::Mat &R,
                                        cv::Mat &t,
                                        const cv::Mat &inliers_mask) {
    cv::Mat K_double, E_double;
    K.convertTo(K_double, CV_64F);
    E.convertTo(E_double, CV_64F);
    cv::recoverPose(E_double, points1, points2, K_double, R, t, inliers_mask);
}

bool triangulate_two_frames(const std::vector<cv::Point2f> &points1,
                            const std::vector<cv::Point2f> &points2,
                            const cv::Mat &T1,
                            const cv::Mat &T2,
                            const cv::Mat &K,
                            std::vector<cv::Point3f> &xyz_points,
                            std::vector<int> &inlier_indices,
                            double min_parallax) {
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

    // find inliers via calculating reprojection errors
    double min_reprojection_error = 1;

    cv::Mat world_to_pixel_proj_mat1 = K * T1;
    cv::Mat world_to_pixel_proj_mat2 = K * T2;
    std::vector<cv::Point3f> reprojected_points1, reprojected_points2;
    bool is_in_front1, is_in_front2;
    std::vector<bool> is_in_front;
    for (int i = 0; i < xyz_points.size(); ++i) {
        reprojected_points1.push_back(world_to_pixel(xyz_points[i], world_to_pixel_proj_mat1, is_in_front1));
        reprojected_points2.push_back(world_to_pixel(xyz_points[i], world_to_pixel_proj_mat2, is_in_front2));
        is_in_front.push_back(is_in_front1 & is_in_front2);
    }

    cv::Mat reprojection_error1 = cv::Mat(reprojected_points1).reshape(1).colRange(0, 2) - cv::Mat(points1).reshape(1);
    cv::Mat reprojection_error2 = cv::Mat(reprojected_points2).reshape(1).colRange(0, 2) - cv::Mat(points2).reshape(1);

//    std::cout << reprojection_error1 << "\n";

    cv::Mat reprojection_error1_norm = cv::Mat::zeros(reprojection_error1.rows, 1, CV_32F);
    cv::Mat reprojection_error2_norm = cv::Mat::zeros(reprojection_error2.rows, 1, CV_32F);
    for (int i = 0; i < reprojection_error1.rows; ++i) {
        reprojection_error1_norm.at<float>(i, 0) = cv::norm(reprojection_error1.row(i), cv::NORM_L2);
        reprojection_error2_norm.at<float>(i, 0) = cv::norm(reprojection_error2.row(i), cv::NORM_L2);
    }

    cv::Mat total_reprojection_error;
    cv::hconcat(reprojection_error1_norm, reprojection_error2_norm, total_reprojection_error);

    cv::Mat mean_reprojection_error;
    cv::reduce(total_reprojection_error, mean_reprojection_error, 1, cv::REDUCE_AVG);

    for (int i =0; i < mean_reprojection_error.rows; ++i) {
//        std::cout << mean_reprojection_error.at<float>(i, 0) << "," << is_in_front[i] << "\n";
        if (is_in_front[i] && (mean_reprojection_error.at<float>(i, 0) < min_reprojection_error))
            inlier_indices.push_back(i);
    }

    std::vector<cv::Point3f> inlier_xyz_points;
    for (const auto &index : inlier_indices) {
        inlier_xyz_points.push_back(xyz_points[index]);
    }

    xyz_points = std::move(inlier_xyz_points);

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
    double min_cos_angle, max_cos_angle;
    cv::minMaxLoc(cos_angles, &min_cos_angle, &max_cos_angle);
    if (min_cos_angle > 0 && max_cos_angle < cos(min_parallax))
        return true;
    return false;
}

#endif // MOTION_ESTIMATION_HPP_