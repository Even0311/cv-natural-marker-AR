#include "PatternDetector.hpp"
#include "DebugHelpers.hpp"
#include <cmath>
#include <iterator>
#include <iostream>
#include <iomanip>
#include <cassert>
#include "macro.h"

void PatternDetector::train(const Pattern& pattern)
{
    m_pattern = pattern;

    m_matcher->clear();

    std::vector<cv::Mat> descriptors(1);
    descriptors[0] = pattern.descriptors.clone();
    m_matcher->add(descriptors);

    m_matcher->train();
}

void PatternDetector::buildPatternFromImage(const cv::Mat& image, Pattern& pattern) const
{
    int numImages = 4;
    float step = sqrtf(2.0f);

    pattern.size = cv::Size(image.cols, image.rows);
    pattern.frame = image.clone();
    getGray(image, pattern.grayImg);

    cv::medianBlur(pattern.grayImg,pattern.grayImg,3);
    pattern.points2d.resize(4);
    pattern.points3d.resize(4);

    const float w = image.cols;
    const float h = image.rows;

    const float maxSize = std::max(w,h);
    const float unitW = w / maxSize;
    const float unitH = h / maxSize;

    pattern.points2d[0] = cv::Point2f(0,0);
    pattern.points2d[1] = cv::Point2f(w,0);
    pattern.points2d[2] = cv::Point2f(w,h);
    pattern.points2d[3] = cv::Point2f(0,h);

    pattern.points3d[0] = cv::Point3f(-unitW, -unitH, 0);
    pattern.points3d[1] = cv::Point3f( unitW, -unitH, 0);
    pattern.points3d[2] = cv::Point3f( unitW,  unitH, 0);
    pattern.points3d[3] = cv::Point3f(-unitW,  unitH, 0);

    extractFeatures(pattern.grayImg, pattern.keypoints, pattern.descriptors);
}



bool PatternDetector::findPattern(const cv::Mat& image, PatternTrackingInfo& info)
{
    getGray(image, m_grayImg);

    TIME_START
    //cv::medianBlur(m_grayImg,m_grayImg,5);
    extractFeatures(m_grayImg, m_queryKeypoints, m_queryDescriptors);

    getMatches(m_queryDescriptors, m_matches);

    bool homographyFound = refineMatchesWithHomography(
            m_queryKeypoints,
            m_pattern.keypoints,
            RANSAC_THRESHOLD,
            m_matches,
            m_roughHomography);

    if (homographyFound)
    {
      //  cv::warpPerspective(m_grayImg, m_warpedImg, m_roughHomography, m_pattern.size, cv::WARP_INVERSE_MAP | cv::INTER_CUBIC);
      //  std::vector<cv::KeyPoint> warpedKeypoints;
      //  std::vector<cv::DMatch> refinedMatches;
      //  extractFeatures(m_warpedImg, warpedKeypoints, m_queryDescriptors);
      //  getMatches(m_queryDescriptors, refinedMatches);
      //  homographyFound = refineMatchesWithHomography(
      //          warpedKeypoints,
      //          m_pattern.keypoints,
      //          RANSAC_THRESHOLD,
      //          refinedMatches,
      //          m_refinedHomography);
      //  info.homography = m_roughHomography * m_refinedHomography;
      //  cv::perspectiveTransform(m_pattern.points2d, info.points2d, info.homography);

         info.homography = m_roughHomography;
        cv::perspectiveTransform(m_pattern.points2d, info.points2d, m_roughHomography);
    }
    TIME_TEST
    return homographyFound;
}

void PatternDetector::getGray(const cv::Mat& image, cv::Mat& gray)
{
    if (image.channels()  == 3)
        cv::cvtColor(image, gray, CV_BGR2GRAY);
    else if (image.channels() == 4)
        cv::cvtColor(image, gray, CV_BGRA2GRAY);
    else if (image.channels() == 1)
        gray = image;
}

bool PatternDetector::extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const
{
    m_detector->detect(image, keypoints);
    if (keypoints.empty())
        return false;

    m_extractor->compute(image, keypoints, descriptors);
    if (keypoints.empty())
        return false;

    return true;
}

void PatternDetector::getMatches(const cv::Mat& queryDescriptors, std::vector<cv::DMatch>& matches)
{
    matches.clear();

    if (enableRatioTest)
    {
        // To avoid NaN's when best match has zero distance we will use inversed ratio. 
        const float minRatio = 1.f / 1.5f;

        // KNN match will return 2 nearest matches for each query descriptor
        m_matcher->knnMatch(queryDescriptors, m_knnMatches, 2);
        //m_matcher->knnMatch(queryDescriptors, m_knnMatches, 2);

        for (size_t i=0; i<m_knnMatches.size(); i++)
        {
            const cv::DMatch& bestMatch   = m_knnMatches[i][0];
            const cv::DMatch& betterMatch = m_knnMatches[i][1];

            float distanceRatio = bestMatch.distance / betterMatch.distance;

            if (distanceRatio < minRatio)
            {
                matches.push_back(bestMatch);
            }
        }
    }
    else
    {
        m_matcher->match(queryDescriptors, matches);
    }
}

bool PatternDetector::refineMatchesWithHomography
        (
                const std::vector<cv::KeyPoint>& queryKeypoints,
                const std::vector<cv::KeyPoint>& trainKeypoints,
                float reprojectionThreshold,
                std::vector<cv::DMatch>& matches,
                cv::Mat& homography
        )
{
    const int MINNUMBER = 8;

    if (matches.size() < MINNUMBER)
        return false;

    std::vector<cv::Point2f> srcPoints(matches.size());
    std::vector<cv::Point2f> dstPoints(matches.size());

    for (size_t i = 0; i < matches.size(); i++)
    {
        srcPoints[i] = trainKeypoints[matches[i].trainIdx].pt;
        dstPoints[i] = queryKeypoints[matches[i].queryIdx].pt;
    }

    std::vector<unsigned char> inliersMask(srcPoints.size());
    homography = cv::findHomography(srcPoints,
                                    dstPoints,
                                    CV_FM_RANSAC,
                                    reprojectionThreshold,
                                    inliersMask);

    std::vector<cv::DMatch> inliers;
    for (size_t i=0; i<inliersMask.size(); i++)
    {
        if (inliersMask[i])
            inliers.push_back(matches[i]);
    }

    matches = std::move(inliers);
    return matches.size() > MINNUMBER;
}
