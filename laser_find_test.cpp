//
// Fuction: track the laser point.
// Created by skywoodsz on 2021/11/6.
//

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d/features2d.hpp"
#include <iostream>
#include <vector>
#include <fstream>

using namespace std;
using namespace cv;

bool Blob_test(Mat &img, Mat &mask)
{
    SimpleBlobDetector::Params params;

    params.thresholdStep = 10;
    params.minThreshold = 50;
    params.maxThreshold = 220;
    params.minRepeatability = 2;
    params.minDistBetweenBlobs = 10;
    params.filterByCircularity = false;
    params.minCircularity = 0.8f;
    params.maxCircularity = (float)3.40282e+038;
    params.filterByInertia = true;
    params.minInertiaRatio = 0.1f;
    params.maxInertiaRatio = (float)3.40282e+038;

    params.filterByColor = false;
    params.blobColor = 0;
    params.filterByArea = true;
    params.minArea = 200;
    params.maxArea = 1000;
    params.filterByConvexity = true;
    params.minConvexity = 0.75f;
    params.maxConvexity = 0.90f;

    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    vector<KeyPoint> key_points;



    detector->detect(mask,key_points);
    drawKeypoints( img, key_points, img, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    drawKeypoints( mask, key_points, mask, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

//    imshow("src", img);
//    waitKey(0);


}


int main()
{

    for (int i = 0; i < 20; ++i) {
        Mat img, gray, mask;
        string path;
        path = "../pic/snap_" + std::to_string(i);
        path += + ".jpg";
        img = imread(path);

        cvtColor(img, gray, COLOR_BGR2GRAY);
        inRange(gray, Scalar(177), Scalar(255), mask);

        Blob_test(img, mask);

        imshow("mask", mask);
        imshow("src", img);
        waitKey(0);
    }


    return 0;

}


