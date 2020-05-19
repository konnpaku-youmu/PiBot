#include "../include/cam_calib/CamCalib.h"

using namespace std;

ExCalibur::ExCalibur(ros::NodeHandle &nh)
{
    // load calibration parameters
    try
    {
        cv::FileStorage cfg("/home/hcrd/PiBot/src/cam_calib/config/calibParas.yaml", cv::FileStorage::READ);
        calibParas.frameNum = (int)cfg["frameCount"];
        calibParas.pattern = (int)cfg["pattern"];
        calibParas.patternRows = (int)cfg["patternSize"]["rows"];
        calibParas.patternCols = (int)cfg["patternSize"]["cols"];
        calibParas.spacing = (float)cfg["patternSize"]["spacing"];
        calibParas.dataSource = (int)cfg["dataSource"];
        calibParas.timeInt = (int)cfg["interval"];
        // assert data integrity

        cfg.release();
        // generate useful params
        patternSize = cv::Size(calibParas.patternCols, calibParas.patternRows);
        status = CAPTURE;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        raise(SIGINT);
    }

    // determine data source
    // 0: ROS message
    // 1: Image folder
    // 2: Live cam
    if (calibParas.dataSource == 0)
    {
        // use ROS
        rawImgSub = nh.subscribe("/rear_camera", 20, &ExCalibur::Calibrate, this);
        calibPub = nh.advertise<sensor_msgs::Image>("/calib", 20);

        // loop
        ros::Rate loop_rate(20);
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    else if (calibParas.dataSource == 1)
    {
        // read from image folder
    }
    else if (calibParas.dataSource == 2)
    {
        // capture live video
        cv::VideoCapture cap;
    }
    else
    {
        ROS_ERROR("Corrupted config file. Abort");
        raise(SIGINT);
    }
}

void ExCalibur::Calibrate(const sensor_msgs::ImageConstPtr rawMsg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImage calibOut;
    cv::Mat rawImg;
    uint32_t timeSinceLastFound;

    // fetch original image
    try
    {
        cv_ptr = cv_bridge::toCvCopy(rawMsg, sensor_msgs::image_encodings::TYPE_8UC1);
        calibOut.header = cv_ptr->header;
        calibOut.encoding = sensor_msgs::image_encodings::BGR8;
        rawImg = cv_ptr->image;
        calibParas.imageSize = cv::Size(rawImg.rows, rawImg.cols);
    }
    catch (const cv_bridge::Exception &e)
    {
        std::cerr << e.what() << '\n';
        raise(SIGINT);
    }

    if (status == CAPTURE && timeSinceLastFound >= calibParas.timeInt)
    {
        // gather images
        bool patternFound;
        vector<cv::Point2f> markers;
        if (calibParas.pattern == 0)
        {
            // find chessboard
            patternFound = cv::findChessboardCorners(rawImg, patternSize,
                                                     markers, CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FAST_CHECK);
            if (patternFound)
            {
                cv::cornerSubPix(rawImg, markers, cv::Size(5, 5), cv::Size(-1, -1),
                                 cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
                cornerPixels.emplace_back(markers);

                vector<cv::Point3f> newObjPoints;
                calcCornerPosition(patternSize, calibParas.spacing, newObjPoints, calibParas.pattern);
                objectPoints.push_back(newObjPoints);

                // visualize
                cv::cvtColor(rawImg, rawImg, CV_GRAY2BGR);
                stringstream text;
                text << "Calibration: " << cornerPixels.size() << "/" << calibParas.frameNum;
                cv::putText(rawImg, text.str(), cv::Point2d(540, 460), CV_FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0, 200, 255), 1, CV_AA);
                cv::drawChessboardCorners(rawImg, patternSize, cv::Mat(markers), patternFound);

                calibOut.image = rawImg;
                calibPub.publish(calibOut.toImageMsg());

                lastFoundStamp = cv_ptr->header.stamp.sec;
            }
        }
        else if (calibParas.pattern == 1)
        {
            // find circle grid
            cv::findCirclesGrid(rawImg, patternSize, markers);
            if (patternFound)
            {
                vector<cv::Point3f> newObjPoints;
                calcCornerPosition(patternSize, calibParas.spacing, newObjPoints, calibParas.pattern);
                objectPoints.push_back(newObjPoints);
            }
        }
        else
        {
            ROS_ERROR("Corrupted config file. Abort");
            raise(SIGINT);
        }
        if (cornerPixels.size() >= calibParas.frameNum)
        {
            status = CALIB;
        }
    }
    else if (status == CALIB)
    {
        // run calibration
        status = runAndSave() ? DONE : CAPTURE;
    }
    else if (status == DONE)
    {
        // show undistorted image
        cv::Mat undistorted;
        cv::cvtColor(rawImg, rawImg, CV_GRAY2BGR);
        cv::undistort(rawImg, undistorted, cameraMat, distCoeffs);
        stringstream text;
        text << "Calibrated! Re-projection Error = " << reProjErr;
        cv::putText(undistorted, text.str(), cv::Point2d(400, 260), CV_FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(100, 240, 0), 1, CV_AA);

        calibOut.image = undistorted;
        calibPub.publish(calibOut.toImageMsg());
    }

    // update time interval
    timeSinceLastFound = cv_ptr->header.stamp.sec - lastFoundStamp;
}

void ExCalibur::calcCornerPosition(cv::Size patternSize, float spacing, vector<cv::Point3f> &corners, int calibPattern)
{
    corners.clear();
    for (int i = 0; i < patternSize.height; ++i)
    {
        for (int j = 0; j < patternSize.width; ++j)
        {
            corners.emplace_back(cv::Point3f(j * calibParas.spacing, i * calibParas.spacing, 0));
        }
    }
}

bool ExCalibur::runAndSave()
{
    vector<cv::Mat> R, t;
    reProjErr = cv::calibrateCamera(objectPoints, cornerPixels, calibParas.imageSize, cameraMat, distCoeffs, R, t);
    if (reProjErr <= 0.5)
    {
        // save camera matrix to file
        cv::FileStorage calibOut("/home/hcrd/PiBot/src/cam_calib/config/camMat.yaml", cv::FileStorage::WRITE);
        
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");

        calibOut << "Calibration Timestamp" << ss.str();
        calibOut << "Camera Matrix" << cameraMat;
        calibOut << "distCoeffs" << distCoeffs;
        calibOut << "Re-projection Error" << reProjErr;
        return true;
    }
    else
    {
        // Too large Re-projection error
        ROS_WARN("Re-projection error out of bound...RE:Capture...");
        objectPoints.clear();
        cornerPixels.clear();
        return false;
    }
}
