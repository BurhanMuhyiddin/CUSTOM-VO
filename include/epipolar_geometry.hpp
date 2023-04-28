#ifndef EPIPOLAR_GEOMETRY_HPP_
#define EPIPOLAR_GEOMETRY_HPP_

#include "opencv2/opencv.hpp"

class EpipolarGeometry
{
public:
    EpipolarGeometry();
    ~EpipolarGeometry();

    static void GetFundamentalMatrix(cv::Mat &F, double &score, cv::Mat &inliers_mask);
    static void GetHomographyMatrix(cv::Mat &H, double &score, cv::Mat &inliers_mask);

    static void SetIntrinsicCameraMatrix(const cv::Mat &K) { K_ = K.clone(); }

    static void SetPoints(const std::vector<cv::Point2f> &points1, const std::vector<cv::Point2f> &points2) {
        points1_ = std::move(points1);
        points2_ = std::move(points2);
    }

private:
    static cv::Mat K_;
    static std::vector<cv::Point2f> points1_, points2_;
    // cv::Mat Ematrix_;
    // cv::Mat Hmatrix_;
};

cv::Mat EpipolarGeometry::K_;
std::vector<cv::Point2f> EpipolarGeometry::points1_;
std::vector<cv::Point2f> EpipolarGeometry::points2_;

EpipolarGeometry::EpipolarGeometry() {
}

EpipolarGeometry::~EpipolarGeometry() {
}

void EpipolarGeometry::GetFundamentalMatrix(cv::Mat &F, double &score, cv::Mat &inliers_mask) {
    // auto E_matrix = cv::findEssentialMat(points1_, points2_, K_, cv::RANSAC, )

    F = cv::findFundamentalMat(points1_, points2_, inliers_mask, cv::RANSAC);

    cv::Mat inlier_points1, inlier_points2;
    cv::Mat inlier_points1_xyz, inlier_points2_xyz;

    for (int i = 0; i < inliers_mask.rows; ++i) {
        if ((int)inliers_mask.at<unsigned char>(i, 0)) {
            inlier_points1.push_back(cv::Point2f(points1_[i]));
            inlier_points2.push_back(cv::Point2f(points2_[i]));

            inlier_points1_xyz.push_back(cv::Point3f(points1_[i].x, points1_[i].y, 1.0));
            inlier_points2_xyz.push_back(cv::Point3f(points2_[i].x, points2_[i].y, 1.0));
        }
    }

    inlier_points1_xyz = inlier_points1_xyz.reshape(1);
    inlier_points2_xyz = inlier_points2_xyz.reshape(1);

    // get F score
    double outlier_threshold = 4;
    cv::Mat epipolar_lines1, error2In1, numerator, denominator, temp;
    cv::computeCorrespondEpilines(inlier_points2, 2, F, epipolar_lines1);
    epipolar_lines1 = epipolar_lines1.reshape(1);
    cv::reduce(inlier_points1_xyz.mul(epipolar_lines1), numerator, 1, cv::REDUCE_SUM);
    cv::multiply(numerator, numerator, numerator);
    temp = epipolar_lines1(cv::Rect(0, 0, 2, epipolar_lines1.rows));
    cv::multiply(temp, temp, temp);
    cv::reduce(temp, denominator, 1, cv::REDUCE_SUM);
    error2In1 = numerator / denominator;

    cv::Mat epipolar_lines2, error1In2;
    cv::computeCorrespondEpilines(inlier_points1, 1, F, epipolar_lines2);
    epipolar_lines2 = epipolar_lines2.reshape(1);
    cv::reduce(inlier_points2_xyz.mul(epipolar_lines2), numerator, 1, cv::REDUCE_SUM);
    cv::multiply(numerator, numerator, numerator);
    temp = epipolar_lines2(cv::Rect(0, 0, 2, epipolar_lines2.rows));
    cv::multiply(temp, temp, temp);
    cv::reduce(temp, denominator, 1, cv::REDUCE_SUM);
    error1In2 = numerator / denominator;

    cv::Mat temp1, temp2;
    cv::reduce(cv::max((outlier_threshold-error1In2), 0), temp1, 0, cv::REDUCE_SUM);
    cv::reduce(cv::max((outlier_threshold-error2In1), 0), temp2, 0, cv::REDUCE_SUM);

    score = cv::Mat(temp1 + temp2).at<float>(0, 0);
}

void EpipolarGeometry::GetHomographyMatrix(cv::Mat &H, double &score, cv::Mat &inliers_mask) {
    H = cv::findHomography(points1_, points2_, inliers_mask, cv::RANSAC, 3.0);
//    H /= H.at<double>(2, 2);

    cv::Mat inlier_points1, inlier_points2;

    for (int i = 0; i < inliers_mask.rows; ++i) {
        if ((int)inliers_mask.at<unsigned char>(i, 0)) {
            inlier_points1.push_back(cv::Point2f(points1_[i]));
            inlier_points2.push_back(cv::Point2f(points2_[i]));
        }
    }

    // Get H score
    double outlier_threshold = 6;
    cv::Mat points1In2, points2In1;
    cv::perspectiveTransform(inlier_points1, points1In2, H);
    cv::perspectiveTransform(inlier_points2, points2In1, H.inv());

    cv::Mat error1In2, error2In1, temp;
    temp = inlier_points2 - points1In2;
    cv::multiply(temp, temp, temp);
    temp = temp.reshape(1);
    cv::reduce(temp, error1In2, 1, cv::REDUCE_SUM);

    temp = inlier_points1 - points2In1;
    cv::multiply(temp, temp, temp);
    temp = temp.reshape(1);
    cv::reduce(temp, error2In1, 1, cv::REDUCE_SUM);

    cv::Mat temp1, temp2;
    cv::reduce(cv::max((outlier_threshold-error1In2), 0), temp1, 0, cv::REDUCE_SUM);
    cv::reduce(cv::max((outlier_threshold-error2In1), 0), temp2, 0, cv::REDUCE_SUM);

    score = cv::Mat(temp1 + temp2).at<float>(0, 0);
}

#endif // EPIPOLAR_GEOMETRY_HPP_