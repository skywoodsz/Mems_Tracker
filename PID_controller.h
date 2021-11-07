//
// Created by skywoodsz on 2021/11/7.
//

#ifndef CPP_SRC_PID_CONTROLLER_H
#define CPP_SRC_PID_CONTROLLER_H
#include<iostream>
class PID_CONTROLLER
{
public:
    PID_CONTROLLER();
    ~PID_CONTROLLER();
    bool PID_CONTROLLER_Init();
    bool PID_CONTROLLER_Data_Conv(double bx, double by);
    bool PID_CONTROLLER_OPen(double bx, double by);
    bool PID_CONTROLLER_Closed(double bx, double by, double lx, double ly);
public:
    double kpx, kdx, kix, kpy, kdy, kiy;
    double ex[3] = {0}, ey[3] = {0};
    double dux, duy;
    double ux, uy;
    
    double motor_x, motor_y;
    double open_ux, open_uy;
    double closed_ux, closed_uy;
    
    double data_point_x1, data_point_x2, data_point_y1, data_point_y2;
    double data_scale_x, data_scale_y;
    
};



#endif //CPP_SRC_SERIAL_PC2MCU_H
