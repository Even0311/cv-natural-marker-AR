#ifndef EXAMPLE_MARKERLESS_AR_PATTERN_HPP
#define EXAMPLE_MARKERLESS_AR_PATTERN_HPP

// File includes:
#include "GeometryTypes.hpp"
#include "CameraInfo.hpp"

#include <opencv2/opencv.hpp>

struct Pattern
{
  cv::Size                  size;
  cv::Mat                   frame;
  cv::Mat                   grayImg;

  std::vector<cv::KeyPoint> keypoints;
  cv::Mat                   descriptors;

  std::vector<cv::Point2f>  points2d;
  std::vector<cv::Point3f>  points3d;
};

struct PatternTrackingInfo
{
  cv::Mat                   homography;
  std::vector<cv::Point2f>  points2d;
  Transformation            pose3d;

  void computePose(const Pattern& pattern, const CameraInfo& camera);
};

#endif