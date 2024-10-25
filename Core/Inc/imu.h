//
// Created by hslnb on 2024/9/19.
//

#ifndef PROJECT_IMU_H
#define PROJECT_IMU_H

#include "main.h"
#include "usart.h"

#ifdef __cplusplus
extern "C"{
#endif

#define FRAME_HEAD 0xfc //起始标志
#define FRAME_END 0xfd  //帧尾
#define imu_data_len 64
#define ahrs_data_len 56
#define gps_data_len 80
//IMU数据
#define TYPE_IMU 0x40   //IMU数据指令类别
#define IMU_LEN  0x38   //56+8  8组数据
//AHRS数据
#define TYPR_AHRS 0x41
#define AHRS_LEN 0x30   //48+8 7组数据
//GPS数据
#define TYPE_GPS 0X42
#define GPS_LEN 0x42    //72+8 10组数据

struct MSG_IMU {
    float gyroscope_x; //机体系X轴角速度 rad/s
    float gyroscope_y; //机体系Y轴角速度 rad/s
    float gyroscope_z; //机体系Z轴角速度 rad/s
    float accelerometer_x; //机体系X轴加速度(未分离重力加速度) m/s^2
    float accelerometer_y; //机体系Y轴加速度(未分离重力加速度) m/s^2
    float accelerometer_z; //机体系Z轴加速度(未分离重力加速度) m/s^2
    float magnetometer_x; //机体系X轴磁感应强度 mG
    float magnetometer_y; //机体系Y轴磁感应强度 mG
    float magnetometer_z; //机体系Z轴磁感应强度[] mG
    float imu_temperature; //如果IMU数据由多个传感器组成则该值为这些传感器的平均温度 C
    float Pressure; //气压值 pa
    float pressure_temperature; //气压计的温度值 C
    uint32_t Timestamp; //时间戳 us
} ;

struct MSG_AHRS {
    float RollSpeed; //横滚角速度 rad/s
    float PitchSpeed; //俯仰角速度 rad/s
    float HeadingSpeed; //偏航角速度 rad/s
    float Roll; //横滚 rad
    float Pitch; //俯仰 rad
    float Heading; //偏航 rad
    float Q1; //四元数Q1
    float Q2; //四元数Q2
    float Q3; //四元数Q3
    float Q4; //四元数Q4
    uint32_t Timestamp; //时间戳 uint:us
} ;

struct MSG_INS_GPS {
    float BodyVelocity_X; //机体系X轴速度
    float BodyVelocity_Y; //机体系Y轴速度
    float BodyVelocity_Z; //机体系Z轴速度
    float BodyAcceleration_X; //机体系X轴加速度
    float BodyAcceleration_Y; //机体系Y轴加速度
    float BodyAcceleration_Z; //机体系Z轴加速度
    float Location_North; //上电为0,从坐标原点到当前向北方向的距离
    float Location_East; //上电为0,从坐标原点到当前向东方向的距离
    float Location_Down; //上电为0,从坐标原点到当前向地方向的距离
    float Velocity_North; //NED系向北速度
    float Velocity_East; //NED系向东速度
    float Velocity_Down; //NED系向地速度
    float Acceleration_North; //NED系向北加速度
    float Acceleration_East; //NED系向东加速度
    float Acceleration_Down; //NED系向地加速度
    float Pressure_Altitude; //由气压计不经过数据融合直接导航出来的高度
    uint32_t Timestamp; //数据的时间戳
} ;

uint8_t CRC8_Table(uint8_t const *p, uint8_t counter);
uint16_t CRC16_Table(uint8_t const *p, uint8_t counter);
#ifdef __cplusplus
//N100惯导数据模块类
class IMU {
public:
    IMU();
    ~IMU();
    struct MSG_IMU* GetIMU();
    struct MSG_AHRS* GetAHRS();
    struct MSG_INS_GPS* GetInsGPS();
    int decodeAHRSPackage(uint8_t *data);
private:
    //解码好的数据
    struct MSG_IMU msg_imu;
    struct MSG_AHRS msg_ahrs;
    struct MSG_INS_GPS msg_ins_gps;
    void handleIMUPackage(int cmd, uint8_t* data, int len);

};
#endif

#ifdef __cplusplus
}
#endif

#endif //PROJECT_IMU_H
