//
// Created by hslnb on 2024/9/19.
//

#ifndef PROJECT_SPIN_H
#define PROJECT_SPIN_H

#include "imu.h"
#include "pid.h"
#include "dma.h"
#include "usart.h"
#include "motor.h"
#include "motion.h"


#define PI 3.141592653589793

struct RobotMotionPosPIDParam
{
    double kp;
    double ki;
    double kd;
    double min_total_error;
    double max_total_error;
};


#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus
namespace RDK{

    class RobotSpin
    {
    public:
        RobotSpin();
        ~RobotSpin();

        double get_current_heading();
        double GetNowHeading();
        double get_now_heading();
        void FixHeading(double heading_needed);
        void spin_to(double heading);
        void spin_left();
        void spin_right();
        void set_lr_speed(double l_speed, double r_speed);
        void stop();
        void LinkMotion(RDK::TwoWheelMotion *twoWheelMotion);
        struct RobotMotionPosPIDParam GetSpin_pid_param();
        void Setset_now_heading(double set_now_heading);

        void SetEncodingMotorLF(EncodingMotor *motor);

        void SetEncodingMotorRF(EncodingMotor *motor);

        void SetEncodingMotorLB(EncodingMotor *motor);

        void SetEncodingMotorRB(EncodingMotor *motor);

    private:
        IMU N100;
        RDK::TwoWheelMotion *twoWheelMotion = nullptr;
        double set_now_heading = 0;
        double max_spin_speed = 60; //Max_spin_speed origin

        struct RobotMotionPosPIDParam spin_pid_param = {6.5, 1.5, 0.15, -1, 1}; //原地旋转PID参数

        EncodingMotor *motorLF = nullptr;
        EncodingMotor *motorRF = nullptr;
        EncodingMotor *motorLB = nullptr;
        EncodingMotor *motorRB = nullptr;
    };
}


#endif

#ifdef __cplusplus
};
#endif


#endif //PROJECT_SPIN_H
