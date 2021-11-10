//
// Created by skywoodsz on 2021/11/10.
//

#ifndef CPP_SRC_CHESSBOARD_H
#define CPP_SRC_CHESSBOARD_H

#include<iostream>
#include<opencv2/opencv.hpp>
#include<string>
#include<vector>

using namespace std;
using namespace cv;

class CHESSBOARD {
public:
    CHESSBOARD(){}
    ~CHESSBOARD(){}
    bool init(vector<Point2f> center, Mat &dst);
    bool AutoInit(Mat &src, Mat &dst);
    bool coordinate(double bx, double by, int &idx);

public:
    vector<Point2f>mc; // center
    vector<vector<Point2f>>corner_vec; // corner

};


#endif //CPP_SRC_CHESSBOARD_H
