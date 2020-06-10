#ifndef CAM_CALIB
#define CAM_CALIB

#include <sstream>
#include <signal.h>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

#define RED cv::Scalar(0, 0, 255)

using namespace std;
struct CalibParasContainer
{
    int frameNum;
    cv::Size imageSize;

    int pattern;
    int patternRows;
    int patternCols;
    float spacing;

    int dataSource;
    int timeInt;
};

enum CALIB_STATUS
{
    CAPTURE = 0,
    CALIB = 1,
    DONE = 2
};

class ExCalibur
{
public:
    ExCalibur(ros::NodeHandle &);

    ~ExCalibur() {}

private:
    ros::Subscriber rawImgSub;

    ros::Publisher calibPub;

    CALIB_STATUS status;

    CalibParasContainer calibParas;

    cv::Size patternSize;

    vector<vector<cv::Point3f>> objectPoints;

    vector<vector<cv::Point2f>> cornerPixels;

    uint32_t lastFoundStamp = 0;

    cv::Mat cameraMat, distCoeffs;

    float reProjErr = 10;

    void Calibrate(const sensor_msgs::ImageConstPtr);

    bool runAndSave();

    void calcCornerPosition(cv::Size, float, vector<cv::Point3f> &, int);
};

#endif
