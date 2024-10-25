//
// Created by hslnb on 2024/9/9.
//

#ifndef FRUIT_ROBOMAIN_H
#define FRUIT_ROBOMAIN_H

#include "main.h"
#include "motion.h"
#include "Spin.h"
#include "motor.h"
#include "Servo.h"
#include "usart.h"

#include <stdbool.h>
#ifdef __cplusplus
extern "C"{
#endif

    extern uint8_t ahrs_data_buffer[1024];
    extern uint8_t ahrs_data_buffer1[1024];
    extern int RxCplt_flag;
////////////////////////底盘运动/////////////////////////////////
void motorCallbackLF(double pwm);
void motorCallbackRF(double pwm);
void motorCallbackLB(double pwm);
void motorCallbackRB(double pwm);
void RobotInit();
void RoboTick();

void RoboTest();
void RobotMoveLineRadius(double lineSpeed, double Radius, bool left);
void RobotMoveSpeed(double linespeed);
void RobotMoveForward(double speed, double ForwardDis);

void RobotClearOdometry();
uint32_t RobotGetOdometry();


void RobotSpinNinety(bool left);
void RobotFixHeading();
void refreshFixHeading();


///////////////////机械臂运动/////////////////////////////////////
void PwmServoCallback1(double pwm_now, double pwm_to_be);
void PwmServoCallback2(double pwm_now, double pwm_to_be);
void RobotInitServo();
void RobotTestServo();

void RobotArmMiddle();
void RobotGrabRightUp();
void RobotGrabRightGround();
void RobotGrabLeftUp();
void RobotGrabLeftGround();
void RobotCGrabLeft();
void RobotCGrabRight();


////////////////全局运动///////////////////////////////////////
void RobotBeginVoice(uint8_t Music, uint8_t *HZdata);


void RobotReceivedFruitVoice();

void RobotEndVoice(uint8_t amount_of_fruits, uint8_t amount_of_vegetables);

void RoboAllMove();


#ifdef __cplusplus
};
#endif

#endif //FRUIT_ROBOMAIN_H

