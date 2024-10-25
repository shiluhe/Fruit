//
// Created by hslnb on 2024/9/9.
//

#ifndef FRUIT_MOTION_H
#define FRUIT_MOTION_H

#include "motor.h"


#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus

#include <utility>//c语言包
#include <cmath>  //c语言包

    /*
     * @brief 4个计米轮
     *  y
     *  |-----------------------|
     *  | motorLF       motorRF |
     *  |                       |
     *  |                       |
     *  |                       |
     *  |                       |
     *  |<----------d---------->|
     *  |                       |
     *  |                       |
     *  |                       |
     *  | motorLB       motorRB |
     *  |-----------------------|--x
     *
     *  小车前方为y轴，左边为x轴
            *  小车线速度 v = (vr + vl) / 2;
    *  小车角速度 w = (vr - vl) / d;
    *  d = 255 mm
            *  记得开四倍频！
    *  正面对着电机 顺时针旋转为正转
    */
namespace RDK{


    class TwoWheelMotion
        {
        private:
            EncodingMotor *motorLF = nullptr;
            EncodingMotor *motorRF = nullptr;
            EncodingMotor *motorLB = nullptr;
            EncodingMotor *motorRB = nullptr; //初始化为指针
            double motorLFSpeed = 0;
            double motorRFSpeed = 0;
            double motorLBSpeed = 0;
            double motorRBSpeed = 0;    // 单位 脉冲/ms
            double LineSpeed = 0;   // 单位为 脉冲/ms
            double AngleSpeed = 0;  // 角速度 单位rad/s，1rad是一个标准量（常量），所以单位也可以是/s
            double Radius = 0;  //输入单位为m 需要转为单位r
            double d = 255;  // 两个轮子之间的距离 单位为mm
            // 在二轮差速模型中，可以直接将r/s替代m/s，结果没有影响
            bool left = true;
            double kp = 0.0710145;
            double ki = 0.010071;
            double kd = 0.00015;
            double minTotalError = -10;
            double maxTotalError = 10;  //积分项的最大最小累计误差
//        RDK::RobotSpin *SpinMotion = nullptr;

        public:
            TwoWheelMotion();

            ~TwoWheelMotion();

            void SetEncodingMotorLF(EncodingMotor *motor);

            void SetEncodingMotorRF(EncodingMotor *motor);

            void SetEncodingMotorLB(EncodingMotor *motor);

            void SetEncodingMotorRB(EncodingMotor *motor);

            void SetLRSpeed(double l_speed, double r_speed);

//        void SetSpinMotion(RobotSpin *SpinMotion);

            void ClearSpeed();

            void AddLineSpeed(double speed);

            void AddAngleSpeed(double speed, bool left);

            void SetWheelDistance(double distance);

            void AddRadius(double r, bool left);

            void CommitSpeed();

            void SetPosPID(double p, double i, double d);

            void Move(double speed, double ForwardDis);

            // 确保 motorLFPosPID 是可访问的
            double GetMotorLFOutput()
            {return motorLFPosPID.GetOutput();}


            // 确保将 motorLFPosPID 声明为类的成员
            PID motorLFPosPID;

            void Stop();

        };

}



#ifdef __cplusplus
}
#endif

#endif
#endif //FRUIT_MOTION_H
