#ifndef HELPER_FUNCTIONS_HPP_
#define HELPER_FUNCTIONS_HPP_

namespace helper
{
    void from_Rt_to_T(const cv::Mat &R, const cv::Mat &t, cv::Mat &T) {
        T = (cv::Mat_<float>(4,4) << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
                                      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
                                      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0),
                                                    0.0,               0.0,               0.0,              1.0);
    }
}

#endif // HELPER_FUNCTIONS_HPP_