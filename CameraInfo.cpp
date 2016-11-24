#include "CameraInfo.hpp"
CameraInfo::CameraInfo(float _fx, float _fy, float _cx, float _cy)
{
    m_intrinsic = cv::Matx33f::zeros();

    fx() = _fx;
    fy() = _fy;
    cx() = _cx;
    cy() = _cy;

    m_distortion.create(5,1);
    for (int i=0; i<5; i++)
        m_distortion(i) = 0;
}

const cv::Matx33f& CameraInfo::getIntrinsic() const
{
    return m_intrinsic;
}

const cv::Mat_<float>&  CameraInfo::getDistorsion() const
{
    return m_distortion;
}

float& CameraInfo::fx()
{
    return m_intrinsic(1,1);
}

float& CameraInfo::fy()
{
    return m_intrinsic(0,0);
}

float& CameraInfo::cx()
{
    return m_intrinsic(0,2);
}

float& CameraInfo::cy()
{
    return m_intrinsic(1,2);
}

float CameraInfo::fx() const
{
    return m_intrinsic(1,1);
}

float CameraInfo::fy() const
{
    return m_intrinsic(0,0);
}

float CameraInfo::cx() const
{
    return m_intrinsic(0,2);
}

float CameraInfo::cy() const
{
    return m_intrinsic(1,2);
}