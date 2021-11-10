//
// Created by skywoodsz on 2021/11/10.
//

#include "chessboard.h"
using namespace std;
using namespace cv;
bool CHESSBOARD::init(vector<Point2f> center, Mat &dst)
{
    mc = center;

    for (int i = 0; i < mc.size(); ++i) {
        putText(dst, to_string(i + 1), mc[i], cv::FONT_HERSHEY_COMPLEX,
                2, cv::Scalar(0, 255, 255), 2, 8, 0);
    }
}
bool CHESSBOARD::AutoInit(Mat &src, Mat &dst)
{
    mc.clear();
    corner_vec.clear();
    cv::Mat gray, thre;
    cvtColor(src, gray, COLOR_RGB2GRAY);
    threshold(gray, thre, 50, 255, THRESH_OTSU);

    vector<vector<Point>> contours, contours_rect;
    vector<Vec4i> hierarchy;
    findContours(thre, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));

    for (int i = 0; i< contours.size(); i++)
    {
        double area = contourArea(contours[i]);
        if(area > 100)
        {
            if(i == 0)
                continue;
            contours_rect.push_back(contours[i]);
        }
    }

    vector<Moments>mu(contours.size());
    for (int i = 0; i < contours_rect.size(); i++) //
    {
        mu[i] = moments(contours_rect[i], false);
    }


    vector<Point2f>corner;
    Point2f pts[4];
    for (int i = 0; i < contours_rect.size(); i++)
    {
        mc.push_back(Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00));

       // Scalar colors = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        RotatedRect rects = minAreaRect(contours_rect[i]);
        rects.points(pts);
        for (int i = 0; i < 4; i++) {
            corner.push_back(pts[i]);
            corner_vec.push_back(corner);
        }
        if (i == 0)
            continue;
        putText(dst, to_string(i), mc[i], cv::FONT_HERSHEY_COMPLEX,
                2, cv::Scalar(0, 255, 255), 2, 8, 0);
    }

    return true;
}
bool CHESSBOARD::coordinate(double bx, double by, int &idx)
{
//    Point2f magin[4];
//    for (int i = 0; i < 4; ++i)
//    {
//        magin[i] = corner_vec[0][i];
//    }
//    bool tempy = (magin[1].y < by) && (by < magin[3].y);
//    bool tempx = (magin[1].x < bx) && (bx < magin[3].x);
//    if (!tempy || !tempx)
//    {
//        idx = -1;
//        cout<<"error! the point is out of the range!";
//        return false;
//    }
    double mindis = std::numeric_limits<double>::max();
    for (int i = 0; i < mc.size(); ++i)
    {
        double dis = pow(bx - mc[i].x, 2) + pow(by - mc[i].y, 2);
        if(dis < mindis)
        {
            mindis = dis;
            idx = i + 1;
        }
    }

    return true;
}