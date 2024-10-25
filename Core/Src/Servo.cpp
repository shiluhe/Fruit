//
// Created by hslnb on 2024/9/18.
//
#include "Servo.h"
#include <cmath>


#define PI 3.141592653589793

Servo::Servo()
{

}

Servo::Servo(ServoType servoType1, double angle, double time, int id)
{
    this->servoType = servoType1;
    this->angle = angle;
    this->time = time;
    this->ID = id;
}

Servo::~Servo()
{

}

void Servo::SetAngle(double angle) {
    this->angle = angle;
}

void Servo::SetTime(double time) {
    this->time = time;
}

void Servo::SetType(ServoType servoType) {
    this->servoType = servoType;
}

void Servo::SetID(int id) {
    this->ID = id;
}

int Servo::GetID() {
    return this->ID;
}

double Servo::GetAngle() {
    return this->angle;
}

double Servo::GetTime() {
    return this->time;
}

ServoType Servo::GetType() {
    return this->servoType;
}

void Servo::SetCallback(PwmServpCallback PwmServoCallback) {
    this->PwmServoCallback = PwmServoCallback;
}

void Servo::PwmCommitAngle(double angle, int angle_range) {
    //周期20ms  PSC=336-1,,ARR=10000-1;
    double zeroDegreePwm = 10000 * 0.5 / 20;
    double maxDegreePwm = 10000 * 2.5 / 20;
    double MinusPwm = maxDegreePwm - zeroDegreePwm;
    double pwm_to_be;
    pwm_to_be = angle / angle_range * MinusPwm + zeroDegreePwm;
    double pwm_now;
    pwm_now = this->angle / angle_range * MinusPwm + zeroDegreePwm;

    if (PwmServoCallback) PwmServoCallback(pwm_now, pwm_to_be);

    this->angle = angle; //更新数字舵机现在的角度;
}

void Servo::MoveBusServoCmd(double angle_to_turn, double time) {
    this->angle = angle_to_turn;
    uint8_t command[MoveBusServoCmdLen];
    uint8_t timeHighByte, timeLowByte;
    this->time = time;
    doubleToHexBytes(this->time, timeHighByte, timeLowByte);
    double Angle = this->angle * 180 / PI / 0.24; //角度位置值 弧度转位置值
    uint8_t angleHighByte, angleLowByte;
    doubleToHexBytes(Angle, angleHighByte, angleLowByte);
    //帧头
    command[0] = 0x55;
    command[1] = 0x55;
    command[2] = this->ID;  //舵机ID号
    command[3] = 7; //数据长度
    command[4] = 1; //指令号
    command[5] = angleLowByte;
    command[6] = angleHighByte;
    command[7] = timeLowByte;
    command[8] = timeHighByte;
    uint8_t params[4]={angleLowByte, angleHighByte, timeLowByte, timeHighByte};
    command[9] = calculateChecksum(command[2], command[3], command[4], params, 4);
    HAL_UART_Transmit(&huart4, command, sizeof(command),HAL_MAX_DELAY);
//    HAL_Delay(10);  //需要适当延时 非阻塞式发送数据不需要(IT or DMA)
//    HAL_Delay(time); //确保舵机完成一个指令的运行 应该是在一整套完成之后再作延迟，这个延迟是没有意义的，因为指令的打断是针对同一舵机
}


void doubleToHexBytes(double decimal, uint8_t &high_byte, uint8_t &low_byte) {
    // 将 double 类型的整数部分转换为16进制表示
    int int_part = (int)round(decimal);  // 只取整数部分

    // 提取高位和低位字节
    high_byte = (int_part >> 8) & 0xFF;
    low_byte = int_part & 0xFF;
}

//double hexBytesToDouble(uint8_t high_byte, uint8_t low_byte) {
//    // 将高位字节和低位字节合并为一个整数
//    int int_part = (high_byte << 8) | low_byte;
//
//    // 将整数转换为 double 类型并返回
//    return static_cast<double>(int_part);
//}

uint8_t calculateChecksum(uint8_t id, uint8_t length, uint8_t cmd, uint8_t const *params, uint8_t param_count) {
    // 计算所有字节的和
    uint16_t sum = id + length + cmd;
    for (uint8_t i = 0; i < param_count; i++) {
        sum += params[i];
    }

    // 取最低的一个字节
    uint8_t low_byte = sum & 0xFF;

    // 取反
    uint8_t checksum = ~low_byte;

    return checksum;
}
