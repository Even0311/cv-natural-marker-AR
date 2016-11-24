#include "ARDrawingContext.hpp"
#include "ARPipeline.hpp"
#include "DebugHelpers.hpp"
#include "CameraInfo.hpp"
#include <cv.h>
#include "macro.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <GLUT/glut.h>
void processVideo(const cv::Mat& patternImage, CameraInfo& camera, cv::VideoCapture& capture);
bool processFrame(const cv::Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx);

static const float f_x = 1056.17169/2;
static const float f_y = 1053.79156/2;
static const float c_x = 626.10138/2;
static const float c_y = 349.58112/2;
bool benchmark = false;
int main(int argc, const char * argv[])
{
    if(benchmark)
    {
        CameraInfo camera(534.4101,531.8235,308.5331,236.0437);

        cv::Mat patternImage = cv::imread("/Users/haoranzhi/pattern_juice.tiff");
        cv::Mat detectImage = cv::imread("/Users/haoranzhi/j2.jpg");
        ARPipeline pipeline(patternImage, camera);
        cv::Size frameSize(detectImage.cols, detectImage.rows);
        ARDrawingContext drawingCtx("AR", frameSize, camera);
        processFrame(detectImage, pipeline, drawingCtx);
        return 0 ;
    }
    CameraInfo camera(f_x,f_y,c_x,c_y);

    cv::Mat patternImage = cv::imread("/Users/haoranzhi/pattern_l.tiff");

    if (patternImage.empty())
    {
        std::cout << "Input image cannot be read" << std::endl;
        return 2;
    }

    cv::VideoCapture cap(0);
    processVideo(patternImage, camera, cap);

    return 0;
}

void processVideo(const cv::Mat& patternImage, CameraInfo& camera, cv::VideoCapture& capture)
{
    cv::Mat currentFrame;
    capture >> currentFrame;
    cv::resize(currentFrame, currentFrame, cv::Size(currentFrame.cols / 2, currentFrame.rows / 2));
    if (currentFrame.empty())
    {
        std::cout << "Cannot open video capture device" << std::endl;
        return;
    }

    cv::Size frameSize(currentFrame.cols, currentFrame.rows);

    ARPipeline pipeline(patternImage, camera);
    ARDrawingContext drawingCtx("AR", frameSize, camera);

    bool shouldQuit = false;
    do
    {
        capture >> currentFrame;

        if (currentFrame.empty())
        {
            shouldQuit = true;
            continue;
        }

        cv::resize(currentFrame, currentFrame, cv::Size(currentFrame.cols / 2, currentFrame.rows / 2));
        shouldQuit = processFrame(currentFrame, pipeline, drawingCtx);
    } while (!shouldQuit);
}

bool processFrame(const cv::Mat& cameraFrame, ARPipeline& pipeline, ARDrawingContext& drawingCtx)
{
    cv::Mat img = cameraFrame.clone();

    cv::putText(img, "RANSAC threshold: " + ToString(pipeline.m_patternDetector.RANSAC_THRESHOLD), cv::Point(10, 10), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(200,200,0));

    drawingCtx.updateBackground(img);
    drawingCtx.isPatternPresent = pipeline.processFrame(cameraFrame);
    drawingCtx.patternPose = pipeline.getPatternLocation();

    // Request redraw of the window:
    drawingCtx.updateWindow();

    // Read the keyboard input:
    int keyCode = cv::waitKey(5);

    bool shouldQuit = false;
    if (keyCode == '+' || keyCode == '=')
    {
        pipeline.m_patternDetector.RANSAC_THRESHOLD+= 0.2f;
        pipeline.m_patternDetector.RANSAC_THRESHOLD= std::min(10.0f, pipeline.m_patternDetector.RANSAC_THRESHOLD);
    }
    else if (keyCode == '-')
    {
        pipeline.m_patternDetector.RANSAC_THRESHOLD-= 0.2f;
        pipeline.m_patternDetector.RANSAC_THRESHOLD= std::max(0.0f, pipeline.m_patternDetector.RANSAC_THRESHOLD);
    }
    else if (keyCode == 27 || keyCode == 'q')
    {
        shouldQuit = true;
    }

    return shouldQuit;
}


