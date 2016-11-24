/*****************************************************************************
*   Markerless AR desktop application.
******************************************************************************
*   by Khvedchenia Ievgen, 5th Dec 2012
*   http://computer-vision-talks.com
******************************************************************************
*   Ch3 of the book "Mastering OpenCV with Practical Computer Vision Projects"
*   Copyright Packt Publishing 2012.
*   http://www.packtpub.com/cool-projects-with-opencv/book
*****************************************************************************/

#ifndef EXAMPLE_MARKERLESS_AR_PATTERNDETECTOR_HPP
#define EXAMPLE_MARKERLESS_AR_PATTERNDETECTOR_HPP

////////////////////////////////////////////////////////////////////
// File includes:
#include "Pattern.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

class PatternDetector
{
public:
    PatternDetector()
            : enableRatioTest(true)
            , RANSAC_THRESHOLD(2)
    {
        m_detector = cv::Ptr<cv::FeatureDetector>(new cv::SiftFeatureDetector());
        m_extractor = cv::Ptr<cv::DescriptorExtractor>(new cv::SiftDescriptorExtractor());
        m_matcher   = cv::Ptr<cv::DescriptorMatcher>(new cv::FlannBasedMatcher());
       // m_detector = cv::Ptr<cv::FeatureDetector>(new cv::ORB(1000));
       // m_extractor = cv::Ptr<cv::DescriptorExtractor>(new cv::FREAK(false, false));
       // m_matcher = cv::Ptr<cv::DescriptorMatcher>(new cv::BFMatcher(cv::NORM_HAMMING,true));
    }
    void train(const Pattern& pattern);

    void buildPatternFromImage(const cv::Mat& image, Pattern& pattern) const;

    bool findPattern(const cv::Mat& image, PatternTrackingInfo& info);

    bool enableRatioTest;
    float RANSAC_THRESHOLD;

protected:

    bool extractFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) const;

    void getMatches(const cv::Mat& queryDescriptors, std::vector<cv::DMatch>& matches);

    static void getGray(const cv::Mat& image, cv::Mat& gray);

    static bool refineMatchesWithHomography(
        const std::vector<cv::KeyPoint>& queryKeypoints, 
        const std::vector<cv::KeyPoint>& trainKeypoints, 
        float reprojectionThreshold,
        std::vector<cv::DMatch>& matches, 
        cv::Mat& homography);

private:
    std::vector<cv::KeyPoint> m_queryKeypoints;
    cv::Mat                   m_queryDescriptors;
    std::vector<cv::DMatch>   m_matches;
    std::vector< std::vector<cv::DMatch> > m_knnMatches;

    cv::Mat                   m_warpedImg;
    cv::Mat                   m_refinedHomography;
    cv::Mat                   m_grayImg;
    cv::Mat                   m_roughHomography;

    Pattern                          m_pattern;
    cv::Ptr<cv::FeatureDetector>     m_detector;
    cv::Ptr<cv::DescriptorExtractor> m_extractor;
    cv::Ptr<cv::DescriptorMatcher>   m_matcher;
};

#endif