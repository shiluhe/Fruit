//
// Created by hslnb on 2024/9/20.
//
#include "Spin.h"
#include <cmath>  // 包含对PI和fmod的定义
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <string.h>

using namespace RDK;

uint8_t ahrs_data_buffer[1024];
uint8_t ahrs_data_buffer1[1024];

int DMA_flag = 0;
int RxCplt_flag = 0;
int msg_flag_Cmd = 0;

RobotSpin::RobotSpin() {}

RobotSpin::~RobotSpin() {}

double RobotSpin::get_current_heading() {//将车头方向规定在360度之间

    while (true) {
        HAL_UART_Receive_DMA(&huart2, ahrs_data_buffer, sizeof(ahrs_data_buffer)); //循环方式开启
        DMA_flag = 1;
        if (DMA_flag == 1) {
            if (RxCplt_flag == 1) {
                if (N100.decodeAHRSPackage(ahrs_data_buffer1)) {
                    RxCplt_flag = 0;
                    break;
                }
            }
        }
    }
    return std::fmod(N100.GetAHRS()->Heading, 2 * PI);
}


void RobotSpin::stop() {
    if (twoWheelMotion) {
        twoWheelMotion->Stop();
    }
}


double RobotSpin::GetNowHeading() {
    return this->get_current_heading();
}

double RobotSpin::get_now_heading() {
    return this->set_now_heading;
}

void RobotSpin::FixHeading(double heading_needed) {
    spin_to(heading_needed);
}

void RobotSpin::Setset_now_heading(double set_now_heading) {
    this->set_now_heading = set_now_heading;
}


void RobotSpin::spin_to(double heading) {
        heading = std::fmod(heading + 2 * PI, 2 * PI);
        heading += 2 * PI;
        RDK::PID spin_pid(RDK::PIDType::Pos, spin_pid_param.kp, spin_pid_param.ki, spin_pid_param.kd); //创建旋转PID对象
        spin_pid.SetTotalErrorRange(spin_pid_param.min_total_error, spin_pid_param.max_total_error); //设置积分误差范围
        spin_pid.SetOutputRange(-max_spin_speed, max_spin_speed);
        spin_pid.SetTarget(heading); //设置PID目标朝向

//    double old_current_heading = get_current_heading();
//    double old_heading = _heading;

    while (true) {
            double current_heading = get_current_heading();
            RxCplt_flag = 0;//get_current_heading()中已经设置为0（按理）

//        _heading = old_heading + current_heading - old_current_heading; //应该是用于机器人自身的姿态判断的

            double heading1 = current_heading; //当前朝向1
            double heading2 = current_heading + 2 * PI; //当前朝向2
            double heading3 = current_heading + 4 * PI; //当前朝向3  增加朝向的原因是，使得角度计算变得合理，找寻最短路径转弯到目标朝向！

            //那个朝向距离目标小就采用哪个朝向
            std::vector<std::pair<double, double>> vec;
            vec.emplace_back(heading1, std::fabs(heading1 - heading));
            vec.emplace_back(heading2, std::fabs(heading2 - heading));
            vec.emplace_back(heading3, std::fabs(heading3 - heading)); //小优化

            std::sort(std::begin(vec), std::end(vec), [](const auto &p1, const auto &p2) {
                //排序，取离朝向距离目标小的
                return p1.second < p2.second;
            });
            current_heading = vec.front().first;

            if (std::fabs(current_heading - heading) < 0.008) break; //达到目标朝向就跳出循环
//        if (timeout <= 0) break;
            spin_pid.SetInput(current_heading);
            spin_pid.Tick();

            double l_speed;
            double r_speed;
            l_speed = -spin_pid.GetOutput();
            r_speed = spin_pid.GetOutput();


            set_lr_speed(l_speed, r_speed);

/////////////////////////Vofa_spinPID////////////////////////////////////////////////////////////////////////////////////////////
//        char bufferLRFB[32];
//        sprintf(bufferLRFB, "%f\n", spin_pid.GetInput());
//        HAL_UART_Transmit(&huart1, (uint8_t *)bufferLRFB, strlen(bufferLRFB),HAL_MAX_DELAY);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }

        stop();
    }

//向左旋转90度
void RobotSpin::spin_left() {
    HAL_UART_Receive_DMA(&huart2, ahrs_data_buffer, sizeof(ahrs_data_buffer)); //循环方式开启
    DMA_flag = 1;
    while (true) {
        if (RxCplt_flag == 1) {//只有串口有接收才能跳出循环
            RxCplt_flag = 0;
            break;
        }
    }
    double current_angle = get_current_heading();
    spin_to(current_angle - PI / 2.0);
}

void RobotSpin::spin_right() {
    HAL_UART_Receive_DMA(&huart2, ahrs_data_buffer, sizeof(ahrs_data_buffer)); //循环方式开启
    DMA_flag = 1;
    while (true) {
        if (RxCplt_flag == 1) {
            RxCplt_flag = 0;
            break;
        }
    }
    double current_angle = get_current_heading();
    spin_to(current_angle + PI / 2.0);
}

void RobotSpin::set_lr_speed(double l_speed, double r_speed) {
    if (twoWheelMotion)
        twoWheelMotion->SetLRSpeed(l_speed, r_speed);
}

void RobotSpin::LinkMotion(RDK::TwoWheelMotion *TwoWheelMotion) {
    this->twoWheelMotion = TwoWheelMotion;
}

struct RobotMotionPosPIDParam RobotSpin::GetSpin_pid_param() {
    return this->spin_pid_param;
}

void RobotSpin::SetEncodingMotorLF(RDK::EncodingMotor *motor) {
    this->motorLF = motor;
}

void RobotSpin::SetEncodingMotorLB(RDK::EncodingMotor *motor) {
    this->motorLB = motor;
}

void RobotSpin::SetEncodingMotorRF(RDK::EncodingMotor *motor) {
    this->motorRF = motor;
}

void RobotSpin::SetEncodingMotorRB(RDK::EncodingMotor *motor) {
    this->motorRB = motor;
}


