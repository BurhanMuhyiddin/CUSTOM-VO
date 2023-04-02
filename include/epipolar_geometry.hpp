#ifndef EPIPOLAR_GEOMETRY_HPP_
#define EPIPOLAR_GEOMETRY_HPP_

#include "opencv2/opencv.hpp"

class EpipolarGeometry
{
public:
    EpipolarGeometry();
    ~EpipolarGeometry();

    void GetEssentialMatrix();
    void GetHomographyMatrix();

    void SetIntrinsicCameraMatrix(const cv::Mat &K) { K_ = K.clone(); }

    void setPoints(const std::vector<cv::Point2f> &points1, const std::vector<cv::Point2f> &points2) {
        points1_ = std::move(points1);
        points2_ = std::move(points2);
    }

private:
    double GetHScore() const;
    double GetEScore() const;

private:
    cv::Mat K_;
    std::vector<cv::Point2f> points1_, points2_;
    // cv::Mat Ematrix_;
    // cv::Mat Hmatrix_;
};

EpipolarGeometry::EpipolarGeometry() {
}

EpipolarGeometry::~EpipolarGeometry() {
}

void EpipolarGeometry::GetEssentialMatrix() {
    auto E_matrix = cv::findEssentialMat(points1_, points2_, K_, cv::RANSAC, )
}

void EpipolarGeometry::GetHomographyMatrix() {

}

double EpipolarGeometry::GetHScore() const {

}

double EpipolarGeometry::GetEScore() const {

}

#endif // EPIPOLAR_GEOMETRY_HPP_