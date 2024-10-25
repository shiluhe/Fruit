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
    //����20ms  PSC=336-1,,ARR=10000-1;
    double zeroDegreePwm = 10000 * 0.5 / 20;
    double maxDegreePwm = 10000 * 2.5 / 20;
    double MinusPwm = maxDegreePwm - zeroDegreePwm;
    double pwm_to_be;
    pwm_to_be = angle / angle_range * MinusPwm + zeroDegreePwm;
    double pwm_now;
    pwm_now = this->angle / angle_range * MinusPwm + zeroDegreePwm;

    if (PwmServoCallback) PwmServoCallback(pwm_now, pwm_to_be);

    this->angle = angle; //�������ֶ�����ڵĽǶ�;
}

void Servo::MoveBusServoCmd(double angle_to_turn, double time) {
    this->angle = angle_to_turn;
    uint8_t command[MoveBusServoCmdLen];
    uint8_t timeHighByte, timeLowByte;
    this->time = time;
    doubleToHexBytes(this->time, timeHighByte, timeLowByte);
    double Angle = this->angle * 180 / PI / 0.24; //�Ƕ�λ��ֵ ����תλ��ֵ
    uint8_t angleHighByte, angleLowByte;
    doubleToHexBytes(Angle, angleHighByte, angleLowByte);
    //֡ͷ
    command[0] = 0x55;
    command[1] = 0x55;
    command[2] = this->ID;  //���ID��
    command[3] = 7; //���ݳ���
    command[4] = 1; //ָ���
    command[5] = angleLowByte;
    command[6] = angleHighByte;
    command[7] = timeLowByte;
    command[8] = timeHighByte;
    uint8_t params[4]={angleLowByte, angleHighByte, timeLowByte, timeHighByte};
    command[9] = calculateChecksum(command[2], command[3], command[4], params, 4);
    HAL_UART_Transmit(&huart4, command, sizeof(command),HAL_MAX_DELAY);
//    HAL_Delay(10);  //��Ҫ�ʵ���ʱ ������ʽ�������ݲ���Ҫ(IT or DMA)
//    HAL_Delay(time); //ȷ��������һ��ָ������� Ӧ������һ�������֮�������ӳ٣�����ӳ���û������ģ���Ϊָ��Ĵ�������ͬһ���
}


void doubleToHexBytes(double decimal, uint8_t &high_byte, uint8_t &low_byte) {
    // �� double ���͵���������ת��Ϊ16���Ʊ�ʾ
    int int_part = (int)round(decimal);  // ֻȡ��������

    // ��ȡ��λ�͵�λ�ֽ�
    high_byte = (int_part >> 8) & 0xFF;
    low_byte = int_part & 0xFF;
}

//double hexBytesToDouble(uint8_t high_byte, uint8_t low_byte) {
//    // ����λ�ֽں͵�λ�ֽںϲ�Ϊһ������
//    int int_part = (high_byte << 8) | low_byte;
//
//    // ������ת��Ϊ double ���Ͳ�����
//    return static_cast<double>(int_part);
//}

uint8_t calculateChecksum(uint8_t id, uint8_t length, uint8_t cmd, uint8_t const *params, uint8_t param_count) {
    // ���������ֽڵĺ�
    uint16_t sum = id + length + cmd;
    for (uint8_t i = 0; i < param_count; i++) {
        sum += params[i];
    }

    // ȡ��͵�һ���ֽ�
    uint8_t low_byte = sum & 0xFF;

    // ȡ��
    uint8_t checksum = ~low_byte;

    return checksum;
}
