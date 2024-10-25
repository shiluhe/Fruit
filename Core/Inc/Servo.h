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

#define MoveBusServoCmdLen 10 //舵机位置控制指令包长度为10
#define ReadBusServoCmdLen 6 //舵机位置读取指令包长度为6
#define BusServoPosBackCmdLen 8 //舵机位置值返回指令包长度为8

using PwmServpCallback = void(*) (double pwm_now, double pwm_to_be);

//舵机类
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
    double angle = 0;   //角度，单位：弧度
    double time = 0;    //时间 单位：毫秒
    int ID;
    uint8_t command_back[BusServoPosBackCmdLen];
    ServoType servoType = BusServo;              //初始化
    PwmServpCallback PwmServoCallback = nullptr; //舵机PWM输出回调函数    舵机执行函数
};

void doubleToHexBytes(double decimal, uint8_t &high_byte, uint8_t &low_byte);   //提取数据的高位与低位并进行存储
double hexBytesToDouble(uint8_t high_byte, uint8_t low_byte); //将获取的高位低位角度字节进行解码
uint8_t calculateChecksum(uint8_t id, uint8_t length, uint8_t cmd, uint8_t const *params, uint8_t param_count); //计算校验码


#endif
#ifdef __cplusplus
}
#endif


#endif //PROJECT_SERVO_H
