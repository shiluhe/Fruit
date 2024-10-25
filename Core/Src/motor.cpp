//
// Created by hslnb on 2024/9/8.
//
#include "motor.h"



namespace RDK {


/*
 * @brief ��������ʼ����ʹ������ʽPID�㷨
 */
    EncodingMotor::EncodingMotor() {
        pid.SetPIDType(RDK::PIDType::Inc);
    }

/*
 * @brief ��ʱΪ��
 */
    EncodingMotor::~EncodingMotor() {

    }

/*
 * @brief ���ñ�������PID����
 * @param p p����ϵ��
 * @param i i����ϵ��
 * @param d d����ϵ��
 */
    void EncodingMotor::SetPID(double p, double i, double d) {
        pid.SetKp(p);
        pid.SetKi(i);
        pid.SetKd(d);
    }

/*
 * @brief ���ñ�������PWM�����Χ
 * @param min ��Сֵ
 * @param max ���ֵ
 */
    void EncodingMotor::SetOutputRange(double min, double max) {
        pid.SetOutputRange(min, max);
    }

/*
 * @brief ���ñ������Ƿ�ת
 * @param reverse �Ƿ�ת
 */
    void EncodingMotor::SetReverse(bool reverse) {
        this->reverse = reverse;
    }

    bool EncodingMotor::GetReverse() {
        return this->reverse;
    }

/*
 * @brief ���ñ�������ת��
 * @param speed ת��
 */
    void EncodingMotor::SetSpeed(double speed) {
        if (this->reverse) speed = -speed;
        pid.SetTarget(speed);
    }

/*
 * @brief ���õ��PWM����ص�����
 * @param motorCallback
 * @details �û�Ӧ��motorCallback�н�PWMֵ��������
 */
    void EncodingMotor::SetCallback(MotorCallback motorCallback) {
        this->motorCallback = motorCallback;
    }

/*
 * @brief ����������
 * @param pulse ������
 * @attention ����ÿ����Tick֮ǰ���ô˷���������������⵽��ת�ٵ��ݸ�PID�㷨
 */
    int EncodingMotor::AddPulse(int pulse) {
        pid.SetInput(pulse);
        if (reverse) pulse = -pulse;
        return this->pulse += pulse;
    }

/*
 * @brief ��ȡ�����������
 */
    int EncodingMotor::GetPulse() volatile {
        return this->pulse;
    }

/*
 * @brief ���������
 */
    int EncodingMotor::ClearPulse() {
        return this->pulse = 0;
    }

/*
 * @brief ����һ��PID���㣬�����PWM�����
 */
    void EncodingMotor::Tick() {

        pid.Tick();//����pid����,input��0.5ms������output��
        if (motorCallback) motorCallback(pid.GetOutput());//���PWM�����
    }

}
