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

#define FRAME_HEAD 0xfc //��ʼ��־
#define FRAME_END 0xfd  //֡β
#define imu_data_len 64
#define ahrs_data_len 56
#define gps_data_len 80
//IMU����
#define TYPE_IMU 0x40   //IMU����ָ�����
#define IMU_LEN  0x38   //56+8  8������
//AHRS����
#define TYPR_AHRS 0x41
#define AHRS_LEN 0x30   //48+8 7������
//GPS����
#define TYPE_GPS 0X42
#define GPS_LEN 0x42    //72+8 10������

struct MSG_IMU {
    float gyroscope_x; //����ϵX����ٶ� rad/s
    float gyroscope_y; //����ϵY����ٶ� rad/s
    float gyroscope_z; //����ϵZ����ٶ� rad/s
    float accelerometer_x; //����ϵX����ٶ�(δ�����������ٶ�) m/s^2
    float accelerometer_y; //����ϵY����ٶ�(δ�����������ٶ�) m/s^2
    float accelerometer_z; //����ϵZ����ٶ�(δ�����������ٶ�) m/s^2
    float magnetometer_x; //����ϵX��Ÿ�Ӧǿ�� mG
    float magnetometer_y; //����ϵY��Ÿ�Ӧǿ�� mG
    float magnetometer_z; //����ϵZ��Ÿ�Ӧǿ��[] mG
    float imu_temperature; //���IMU�����ɶ��������������ֵΪ��Щ��������ƽ���¶� C
    float Pressure; //��ѹֵ pa
    float pressure_temperature; //��ѹ�Ƶ��¶�ֵ C
    uint32_t Timestamp; //ʱ��� us
} ;

struct MSG_AHRS {
    float RollSpeed; //������ٶ� rad/s
    float PitchSpeed; //�������ٶ� rad/s
    float HeadingSpeed; //ƫ�����ٶ� rad/s
    float Roll; //��� rad
    float Pitch; //���� rad
    float Heading; //ƫ�� rad
    float Q1; //��Ԫ��Q1
    float Q2; //��Ԫ��Q2
    float Q3; //��Ԫ��Q3
    float Q4; //��Ԫ��Q4
    uint32_t Timestamp; //ʱ��� uint:us
} ;

struct MSG_INS_GPS {
    float BodyVelocity_X; //����ϵX���ٶ�
    float BodyVelocity_Y; //����ϵY���ٶ�
    float BodyVelocity_Z; //����ϵZ���ٶ�
    float BodyAcceleration_X; //����ϵX����ٶ�
    float BodyAcceleration_Y; //����ϵY����ٶ�
    float BodyAcceleration_Z; //����ϵZ����ٶ�
    float Location_North; //�ϵ�Ϊ0,������ԭ�㵽��ǰ�򱱷���ľ���
    float Location_East; //�ϵ�Ϊ0,������ԭ�㵽��ǰ�򶫷���ľ���
    float Location_Down; //�ϵ�Ϊ0,������ԭ�㵽��ǰ��ط���ľ���
    float Velocity_North; //NEDϵ���ٶ�
    float Velocity_East; //NEDϵ���ٶ�
    float Velocity_Down; //NEDϵ����ٶ�
    float Acceleration_North; //NEDϵ�򱱼��ٶ�
    float Acceleration_East; //NEDϵ�򶫼��ٶ�
    float Acceleration_Down; //NEDϵ��ؼ��ٶ�
    float Pressure_Altitude; //����ѹ�Ʋ����������ں�ֱ�ӵ��������ĸ߶�
    uint32_t Timestamp; //���ݵ�ʱ���
} ;

uint8_t CRC8_Table(uint8_t const *p, uint8_t counter);
uint16_t CRC16_Table(uint8_t const *p, uint8_t counter);
#ifdef __cplusplus
//N100�ߵ�����ģ����
class IMU {
public:
    IMU();
    ~IMU();
    struct MSG_IMU* GetIMU();
    struct MSG_AHRS* GetAHRS();
    struct MSG_INS_GPS* GetInsGPS();
    int decodeAHRSPackage(uint8_t *data);
private:
    //����õ�����
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
