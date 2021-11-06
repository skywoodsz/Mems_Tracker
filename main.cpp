/*
 * 功能：Blob and Laser tracker.
 * Created by ZhangTianlin on 2021/11/6.
 */

#include <iostream>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d/features2d.hpp"

#include "serial_pc2mcu.h"

using namespace std;
using namespace cv;

#define SER

// 拍摄截图
bool Get_Snap(Mat img, int i)
{
    Mat dst = img.clone();

    string path;
    path = "../pic/snap_" + std::to_string(i);
    path += + ".jpg";
    std::cout<<path<<std::endl;
    imwrite(path, dst);
}

// 绘制追踪圆
bool Draw_Init_Circle()
{
    Mat back(1280, 720, CV_8UC1, Scalar(255));
    // -1: full
    circle(back, Point(back.cols/ 2, back.rows / 2), 30, Scalar(0, 0, 255), -1);
    imwrite("./Tracking_Circle.jpg", back);
    return true;
}

/*
 * 1. black blob detect
 * 2. laser point detect
 * 3. buffer send
*/
int main() {
    // 0. port
    #ifdef SER
        Serial_PC2MCU serial;
        string port = "/dev/stm32"; //COM need to fix
        if(!serial.init(port, 15200))
        {
            exit(0);
        }
    #endif

    // 0. 相机校正
    const cv::Mat K = ( cv::Mat_<double> ( 3,3 )
            << 1259.157807, 0, 626.587968, 0, 1259.207534, 323.224541, 0, 0, 1);
    const cv::Mat D = ( cv::Mat_<double> ( 5,1 )
            << -0.396304, 0.324814, -0.003378, -0.002047, 0.000000);

    // Blob Detctor param
    SimpleBlobDetector::Params params;
    params.thresholdStep = 10;
    params.minThreshold = 50;
    params.maxThreshold = 220;
    params.minRepeatability = 2;
    params.minDistBetweenBlobs = 10;
    params.filterByColor = true;
    params.blobColor = 0;
    params.filterByArea = true;
    params.minArea = 100;
    params.maxArea = 50000;
    params.filterByCircularity = false;
    params.minCircularity = 0.8f;
    params.maxCircularity = (float)3.40282e+038;
    params.filterByInertia = true;
    params.minInertiaRatio = 0.1f;
    params.maxInertiaRatio = (float)3.40282e+038;
    params.filterByConvexity = true;
    params.minConvexity = 0.95f;
    params.maxConvexity = std::numeric_limits<float>::max();

    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

    // Laser Detctor param
    params.filterByColor = false;
    params.blobColor = 0;
    params.filterByArea = true;
    params.minArea = 200;
    params.maxArea = 1000;
    params.filterByConvexity = true;
    params.minConvexity = 0.55f;
    params.maxConvexity = 0.90f;

    Ptr<SimpleBlobDetector> detector_laser = SimpleBlobDetector::create(params);

    vector<KeyPoint> Black_Circle_Key_Points, Laser_Key_Points;

    Mat frame, gray, mask;
    int bx, by, lx, ly;

    VideoCapture capture(1);
    int i = 0;
    while (true)
    {
        capture >> frame;
        if(frame.empty())
        {
            std::cout<<"Can't capture the video!"<<std::endl;
            exit(0);
        }
        // 0. 校正
        cv::Mat UndistortImage;
        cv::undistort(frame, UndistortImage, K, D, K);

        // 1. blob detcor
        detector->detect(UndistortImage,Black_Circle_Key_Points);
        drawKeypoints( UndistortImage, Black_Circle_Key_Points, UndistortImage, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

        if(Black_Circle_Key_Points.size() != 0)
        {
            std::cout<<"Black_Circle_Key_Points: "<<Black_Circle_Key_Points[0].pt<<std::endl;
            bx = Black_Circle_Key_Points[0].pt.x;
            by = Black_Circle_Key_Points[0].pt.y;
            circle(UndistortImage, Black_Circle_Key_Points[0].pt, 5, Scalar(255, 0, 0), -1);
        }

        // 2. laser point detect
        cvtColor(UndistortImage, gray, COLOR_BGR2GRAY);
        inRange(gray, Scalar(177), Scalar(255), mask);

        detector_laser->detect(mask,Laser_Key_Points);
        drawKeypoints( UndistortImage, Laser_Key_Points, UndistortImage, Scalar(0,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

        if(Laser_Key_Points.size() != 0)
        {
            std::cout<<"Laser_Key_Points: "<<Laser_Key_Points[0].pt<<std::endl;
            lx = Laser_Key_Points[0].pt.x;
            ly = Laser_Key_Points[0].pt.y;
            circle(UndistortImage, Laser_Key_Points[0].pt, 5, Scalar(255, 0, 0), -1);
        }

        namedWindow("SimpleBlobDetector");
        imshow("SimpleBlobDetector", UndistortImage);

        if(waitKey(20) == 'q')
            break;

        // 3. send data to mcu
        #ifdef SER
        if(Black_Circle_Key_Points.size() != 0 && Laser_Key_Points.size() != 0)
        {
            serial.send_data (bx, by, lx, ly);
            std::cout<<"send the data!"<<std::endl;
        }
        #endif



//        if(waitKey(10) == 'w')
//        {
//            Get_Snap(frame, i);
//            std::cout<<"snap the "<<i<<" pic"<<std::endl;
//            i++;
//        }
    }

    capture.release();
    destroyAllWindows();
    return 0;

}

