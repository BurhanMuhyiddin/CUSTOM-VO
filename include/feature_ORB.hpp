#ifndef FEATUREORB_HPP_
#define FEATUREORB_HPP_

#include <algorithm>
#include <opencv2/features2d/features2d.hpp>

class FeatureExtractor {
public:
    FeatureExtractor() {
        detector_ = cv::ORB::create(1000, 1.2f, 8, 31, 0, 2, cv::ORB::FAST_SCORE);
        descriptor_ = cv::ORB::create();
    }

    void Extract(const cv::Mat &img) {
        detector_->detect(img, keypoints_);
        descriptor_->compute(img, keypoints_, descriptors_);
    }

    cv::Mat getDescriptor() const { return descriptors_; }

    std::vector<cv::KeyPoint>& getKeypoints() { return keypoints_; }

private:
    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> descriptor_;
    cv::Mat descriptors_;
    std::vector<cv::KeyPoint> keypoints_;
};

class FeatureMatcher {
public:
    FeatureMatcher() {
        matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
    }

    void Match(const cv::Mat &img1, const cv::Mat &img2) {
        img1_ = img1;
        img2_ = img2;

        ext1_.Extract(img1);
        ext2_.Extract(img2);
        
        std::vector<cv::DMatch> matches_;
        matcher_->match(ext1_.getDescriptor(), ext2_.getDescriptor(), matches_);
        FilterMatches(matches_);
    }

    cv::Mat DrawMatches() {
        cv::Mat img_match;
        cv::drawMatches(img1_, ext1_.getKeypoints(), img2_, ext2_.getKeypoints(), filtered_matches_, img_match);
        return img_match;
    }

    std::vector<cv::DMatch> getMatches() const { return filtered_matches_; }

    std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> GetMatchedPoints() {
        std::vector<cv::Point2f> points1;
        std::vector<cv::Point2f> points2;

        auto keypoints_1 = ext1_.getKeypoints();
        auto keypoints_2 = ext2_.getKeypoints();

        for (int i = 0; i < (int) filtered_matches_.size(); ++i) {
            points1.push_back(keypoints_1[filtered_matches_[i].queryIdx].pt);
            points2.push_back(keypoints_2[filtered_matches_[i].trainIdx].pt);
        }

        return { points1, points2 };
    }

    size_t GetNumMatchedPoints() const { return filtered_matches_.size(); }

private:
    void FilterMatches(const std::vector<cv::DMatch> &matches) {
        filtered_matches_.clear();

        auto min_max = std::minmax_element(matches.begin(), matches.end(),
                                           [](const cv::DMatch &m1, const cv::DMatch &m2) { return m1.distance < m2.distance; });
        
        double min_dist = min_max.first->distance;
        double max_dist = min_max.second->distance;

        for (size_t i = 0; i < matches.size(); ++i) {
            if (matches[i].distance < std::max(2*min_dist, 30.0)) {
                filtered_matches_.push_back(matches[i]);
            }
        }
    }
private:
    FeatureExtractor ext1_;
    FeatureExtractor ext2_;

    cv::Mat img1_;
    cv::Mat img2_;

    cv::Ptr<cv::DescriptorMatcher> matcher_;
    std::vector<cv::DMatch> filtered_matches_;
};

#endif // FEATUREORB_HPP_