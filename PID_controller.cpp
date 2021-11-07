//
// Created by skywoodsz on 2021/11/7.
//

#include "PID_controller.h"

PID_CONTROLLER::PID_CONTROLLER(){}

PID_CONTROLLER::~PID_CONTROLLER(){}

bool PID_CONTROLLER::PID_CONTROLLER_Init()
{
    kpx = 0;
    kdx = 0;
    kix = 0;
    kpy = 0;
    kdy = 0;
    kiy = 0;
    
    // ex[3] = {0};
    // ey[3] = {0};
    dux = 0;
    duy = 0;
    ux = 0;
    uy = 0;
    
    motor_x = 0;
    motor_y = 0;
    open_ux = 0;
    open_uy = 0;
    closed_ux = 0;
    closed_uy = 0; 
    
    data_point_x1 = -2600;
    data_point_x2 = 2600;
    data_point_y1 = -300;
    data_point_y2 = 300;
    
    return true;
    
}

// data conv: pixel space -> motor space
bool PID_CONTROLLER::PID_CONTROLLER_Data_Conv(double bx, double by)
{
    data_scale_x = data_point_x2 - data_point_x1;
    data_scale_y = data_point_y2 - data_point_y1;
    
    motor_x = bx / 1920 * data_scale_x  + data_point_x1;
    motor_y = by / 1080 * data_scale_y  + data_point_y1;
    
    return true;
    
}

// open controller
bool PID_CONTROLLER::PID_CONTROLLER_OPen(double bx, double by)
{
    PID_CONTROLLER_Data_Conv(bx, by);
    
    open_ux = motor_x;
    open_uy = motor_y;
    
    return true;
}

// zengliang closed controller
bool PID_CONTROLLER::PID_CONTROLLER_Closed(double bx, double by, double lx, double ly)
{
    // current error
    // e[0]: current error; e[1]: last error; e[2]: previous error 
    ex[0] = (lx - bx);
    ey[0] = (ly - by);
    
    dux = kpx * (ex[0] - ex[1]) + kix * ex[0] + kdx * (ex[0] - 2 * ex[1] + ex[2]);
    
    duy = kpy * (ey[0] - ey[1]) + kiy * ey[0] + kdy * (ey[0] - 2 * ey[1] + ey[2]);
    
    // update control
    ux += dux;
    uy += duy;
    
    //update error
    ex[1] = ex[0];
    ex[2] = ex[1];
    
    ey[1] = ey[0];
    ey[2] = ey[1];
    
    return true;
}





