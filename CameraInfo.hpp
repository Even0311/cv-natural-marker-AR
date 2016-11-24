#ifndef __CAMERA_INFO__
#define __CAMERA_INFO__

////////////////////////////////////////////////////////////////////
// File includes:
#include <opencv2/opencv.hpp>

/**
* A camera calibration class that stores intrinsic matrix and distortion coefficients.
*/
class CameraInfo
{
public:
    CameraInfo(float fx, float fy, float cx, float cy);

    const cv::Matx33f& getIntrinsic() const;
    const cv::Mat_<float>&  getDistorsion() const;

    float& fx();
    float& fy();

    float& cx();
    float& cy();

    float fx() const;
    float fy() const;

    float cx() const;
    float cy() const;
private:
    cv::Matx33f     m_intrinsic;
    cv::Mat_<float> m_distortion;
};

#endif
