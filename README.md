## 1. 运行平台  
ubuntu1604 + opencv4.0.0 + boost1.58.  

## 2. 功能  
实现黑色blob与实时激光点的追踪，并向下位机发送blob与laser的像素坐标.  

## 3. 架构  
- pic: 测试图像  
- Tracking_Circle_XXX: 追踪blob图像  
- main.cpp: 主函数  
- serial_pc2mcu.cpp/h: 串口通信封装类  
- sys_test.cpp: 基于测试图像的函数测试  
- laser_find_test.cpp: laser追踪点调试  
- PID_controller.cpp/h: pid controller

