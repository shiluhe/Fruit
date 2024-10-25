//
// Created by hslnb on 2024/9/18.
//

#ifndef PROJECT_SERVO_H
#define PROJECT_SERVO_H

#include "main.h"
#include "usart.h"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
typedef enum {
    PwmServo = 0,
    BusServo = 1
} ServoType;

#define MoveBusServoCmdLen 10 //���λ�ÿ���ָ�������Ϊ10
#define ReadBusServoCmdLen 6 //���λ�ö�ȡָ�������Ϊ6
#define BusServoPosBackCmdLen 8 //���λ��ֵ����ָ�������Ϊ8

using PwmServpCallback = void(*) (double pwm_now, double pwm_to_be);

//�����
class Servo {
public:
    Servo();

    Servo(ServoType servoType, double angle, double time, int id);

    ~Servo();

    double GetAngle();

    double GetTime();

    ServoType GetType();

    int GetID();

    void SetType(ServoType servoType);

    void SetAngle(double angle);

    void SetTime(double time);

    void SetID(int id);

    void MoveBusServoCmd(double angle, double time);

    //void ReadBusServoCmd();

    void SetCallback(PwmServpCallback PwmServoCallback);

    void PwmCommitAngle(double angle, int angle_range);
private:
    double angle = 0;   //�Ƕȣ���λ������
    double time = 0;    //ʱ�� ��λ������
    int ID;
    uint8_t command_back[BusServoPosBackCmdLen];
    ServoType servoType = BusServo;              //��ʼ��
    PwmServpCallback PwmServoCallback = nullptr; //���PWM����ص�����    ���ִ�к���
};

void doubleToHexBytes(double decimal, uint8_t &high_byte, uint8_t &low_byte);   //��ȡ���ݵĸ�λ���λ�����д洢
double hexBytesToDouble(uint8_t high_byte, uint8_t low_byte); //����ȡ�ĸ�λ��λ�Ƕ��ֽڽ��н���
uint8_t calculateChecksum(uint8_t id, uint8_t length, uint8_t cmd, uint8_t const *params, uint8_t param_count); //����У����


#endif
#ifdef __cplusplus
}
#endif


#endif //PROJECT_SERVO_H
