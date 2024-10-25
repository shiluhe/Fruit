//
// Created by hslnb on 2024/9/9.
//
#include "motion.h"
#include "cstdio"
#include "usart.h"
#include "string.h"


extern UART_HandleTypeDef huart1;
namespace RDK {
    TwoWheelMotion::TwoWheelMotion(){}

    TwoWheelMotion::~TwoWheelMotion(){}

    void TwoWheelMotion::SetEncodingMotorLF(RDK::EncodingMotor *motor) {
        this->motorLF = motor;
    }

    void TwoWheelMotion::SetEncodingMotorRF(RDK::EncodingMotor *motor) {
        this->motorRF = motor;
    }

    void TwoWheelMotion::SetEncodingMotorLB(RDK::EncodingMotor *motor) {
        this->motorLB = motor;
    }

    void TwoWheelMotion::SetEncodingMotorRB(RDK::EncodingMotor *motor) {
        this->motorRB = motor;
    }

    void TwoWheelMotion::ClearSpeed() {
        motorLFSpeed = 0;
        motorRFSpeed = 0;
        motorLBSpeed = 0;
        motorRBSpeed = 0;
    }

    void TwoWheelMotion::AddLineSpeed(double speed) {
        this->LineSpeed += speed;
        this->motorLFSpeed += speed;
        this->motorRFSpeed += speed;
        this->motorLBSpeed += speed;
        this->motorRBSpeed += speed;
    }

    void TwoWheelMotion::AddAngleSpeed(double speed, bool left) {
        this->AngleSpeed += speed;
        if (left) {
            this->motorLFSpeed -= 0.5 * (this->d / 300 * 60000 / 1000) * speed;
            this->motorLBSpeed -= 0.5 * (this->d / 300 * 60000 / 1000) * speed;
            this->motorRFSpeed += 0.5 * (this->d / 300 * 60000 / 1000) * speed;
            this->motorRBSpeed += 0.5 * (this->d / 300 * 60000 / 1000) * speed;
        } else {
            this->motorLFSpeed += 0.5 * (this->d / 300 * 60000 / 1000) * speed;
            this->motorLBSpeed += 0.5 * (this->d / 300 * 60000 / 1000) * speed;
            this->motorRFSpeed -= 0.5 * (this->d / 300 * 60000 / 1000) * speed;
            this->motorRBSpeed -= 0.5 * (this->d / 300 * 60000 / 1000) * speed;
        }

    }

    void TwoWheelMotion::SetWheelDistance(double distance) {
        this->d = distance;
    }

    void TwoWheelMotion::AddRadius(double r, bool left) {
        if (left) {
            if (motorLF)
                motorLF->SetReverse(false);
            if (motorLB)
                motorLB->SetReverse(false);
            if (motorRB)
                motorRB->SetReverse(false);
            if (motorRF)
                motorRF->SetReverse(false);
        } else {
            if (motorLF)
                motorLF->SetReverse(true);
            if (motorLB)
                motorLB->SetReverse(true);
            if (motorRB)
                motorRB->SetReverse(true);
            if (motorRF)
                motorRF->SetReverse(true);
        }

        double addAngleSpeed = 0;
        double NewAngleSpeed = 0;
        this->Radius += r / 300 * 60000;
        NewAngleSpeed = this->LineSpeed / this->Radius * 1000; // s- ʹ�����������w
        addAngleSpeed = NewAngleSpeed - this->AngleSpeed;
        this->AddAngleSpeed(addAngleSpeed, left);
    }

    void TwoWheelMotion::CommitSpeed()
    {
        if (motorRB)
            motorRB->SetSpeed(motorRBSpeed);
        if (motorLF)
            motorLF->SetSpeed(motorLFSpeed);
        if (motorRF)
            motorRF->SetSpeed(motorRFSpeed);
        if (motorLB)
            motorLB->SetSpeed(motorLBSpeed);
    }

    void TwoWheelMotion::SetPosPID(double p, double i, double d) {
        this->kp = p;
        this->ki = i;
        this->kd = d;
    }

    /*
  * @brief ���ֲ���ǰ���˶�
  * @attention ���ô˷�����һֱ�������ƶ�������
  * @param speed �ƶ��ٶ�
  * @param ForwardDis ǰ������
  */
    double TwoWheelMotion::Move(double speed, double ForwardDis) {
        PID motorRBPosPID(RDK::PIDType::Pos, this->kp, this->ki, this->kd);//ʹ��λ��ʽPID���п���ʹ��С���������ƶ�
        PID motorRFPosPID(RDK::PIDType::Pos, this->kp, this->ki, this->kd);//ʹ��λ��ʽPID���п���ʹ��С���������ƶ�
        PID motorLBPosPID(RDK::PIDType::Pos, this->kp, this->ki, this->kd);//ʹ��λ��ʽPID���п���ʹ��С���������ƶ�
        PID motorLFPosPID(RDK::PIDType::Pos, this->kp, this->ki, this->kd);//ʹ��λ��ʽPID���п���ʹ��С���������ƶ�

        if (speed < 0) speed = -speed;
        motorRBPosPID.SetTotalErrorRange(minTotalError, maxTotalError);
        motorRFPosPID.SetTotalErrorRange(minTotalError, maxTotalError);
        motorLBPosPID.SetTotalErrorRange(minTotalError, maxTotalError);
        motorLFPosPID.SetTotalErrorRange(minTotalError, maxTotalError);//�����ۼ���Χ

        motorRBPosPID.SetOutputRange(-speed, speed);
        motorRFPosPID.SetOutputRange(-speed, speed);
        motorLBPosPID.SetOutputRange(-speed, speed);
        motorLFPosPID.SetOutputRange(-speed, speed);//���������Χ

        //�ȴ����ֱ����������ת�䣬����ForwardDis��Ŀ��������
        double motorRBTargetDis = ForwardDis;
        double motorLBTargetDis = ForwardDis;
        double motorRFTargetDis = ForwardDis;
        double motorLFTargetDis = ForwardDis;
        //ԭ����������0��������һ���������˶�
        this->motorRB->ClearPulse();
        this->motorRF->ClearPulse();
        this->motorLB->ClearPulse();
        this->motorLF->ClearPulse();

        this->motorLF->SetReverse(false);
        this->motorLB->SetReverse(false);
        this->motorRF->SetReverse(true);
        this->motorRB->SetReverse(true);

        motorRBPosPID.SetTarget(motorRBTargetDis);  //����Ŀ���������������
        motorRFPosPID.SetTarget(motorRFTargetDis);  //����Ŀ���������������
        motorLBPosPID.SetTarget(motorLBTargetDis);  //����Ŀ���������������
        motorLFPosPID.SetTarget(motorLFTargetDis);  //����Ŀ���������������

        int timeout = ForwardDis * 0.75; //��ʱ����������ֹ��������
        while (true)
        {
            if (std::fabs(motorLF->GetPulse() - motorLFTargetDis) < 5 &&
                std::fabs(motorLB->GetPulse() - motorLBTargetDis) < 5 &&
                std::fabs(motorRF->GetPulse() - motorRFTargetDis) < 5 &&
                std::fabs(motorRB->GetPulse() - motorRBTargetDis) < 5)
            {
                break;
            }
            if (timeout <= 0) {
                break;
            }

            motorRBPosPID.SetInput(motorRB->GetPulse());//����PID
            motorRBPosPID.Tick();
            motorLBPosPID.SetInput(motorLB->GetPulse());
            motorLBPosPID.Tick();
            motorRFPosPID.SetInput(motorRF->GetPulse());
            motorRFPosPID.Tick();
            motorLFPosPID.SetInput(motorLF->GetPulse());
            motorLFPosPID.Tick();


            ClearSpeed();
            motorRBSpeed += motorRBPosPID.GetOutput();
            motorLBSpeed += motorLBPosPID.GetOutput();
            motorRFSpeed += motorRFPosPID.GetOutput();
            motorLFSpeed += motorLFPosPID.GetOutput();


          printf("LF:%f RF:%f\n", motorLFPosPID.GetOutput(),motorRFPosPID.GetOutput());

            CommitSpeed();
            timeout--;

        }
        ClearSpeed();
        CommitSpeed();
        //Stop();
       //return flag;
    }

    void TwoWheelMotion::Stop()
    {
        ClearSpeed();
        CommitSpeed();
        if (motorLF) motorLF->SetReverse(false);
        if (motorLB) motorLB->SetReverse(false);
        if (motorRF) motorRF->SetReverse(true);
        if (motorRB) motorRB->SetReverse(true); //׼��ֱ��
    }

    void TwoWheelMotion::SetLRSpeed(double l_speed, double r_speed) {
        if (l_speed < 0 && r_speed > 0) {   //����
            l_speed = -l_speed;
            if (motorLF)
                motorLF->SetReverse(true);
            if (motorLB)
                motorLB->SetReverse(true);
            if (motorRB)
                motorRB->SetReverse(true);
            if (motorRF)
                motorRF->SetReverse(true);
        } else if (r_speed < 0 && l_speed > 0) {  //����
            r_speed = -r_speed;
            if (motorLF)
                motorLF->SetReverse(false);
            if (motorLB)
                motorLB->SetReverse(false);
            if (motorRB)
                motorRB->SetReverse(false);
            if (motorRF)
                motorRF->SetReverse(false);
        }

        motorLFSpeed = l_speed;
        motorLBSpeed = l_speed;
        motorRFSpeed = r_speed;
        motorRBSpeed = r_speed;
        CommitSpeed();
    }

}

