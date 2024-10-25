//
// Created by hslnb on 2024/9/9.
//
#include "RoboMain.h"
#include "string.h" //ʹ��strlen��Ҫ
#include "stdio.h"  // ���� sprintf ����


#define PI 3.141592653589793
uint8_t amount_of_fruits = 0;
uint8_t amount_of_vegetables = 0;
uint8_t single_byte;
uint8_t k210_msg[18];
volatile int k210_msg_index = 0;
int k210_msg_flag = 0;
int rx_times_flag1 = 0;
int rx_times_flag2 = 0;
int msg_flag = 0;

int k210_msg_received_flag = 0;

uint8_t done_msg[4];

#define DATA_BUFFER_SIZE 64
uint8_t dataBuffer[DATA_BUFFER_SIZE];
uint8_t distanceData[2]; // �������ݿ�����Ҫ�����ֽ�
////�����˶�/////////////////////////////////////////////////////
RDK::EncodingMotor encodingMotorLF;
RDK::EncodingMotor encodingMotorRF;
RDK::EncodingMotor encodingMotorLB;
RDK::EncodingMotor encodingMotorRB;
RDK::TwoWheelMotion twoWheelMotion;
RDK::RobotSpin SpinMotion;

double forward_heading_fix = 0;

//��������ת��PWM
void motorCallbackLF(double pwm)
{
    if (encodingMotorLF.GetReverse())
    {
        pwm = -pwm;
        if(pwm < 0)
        {
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000+pwm);
        }
        else if (pwm > 0)
        {
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm);
        }
        else if(pwm == 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
        }
    }
    else
    {
        if(pwm < 0)
        {
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 1000+pwm);
        }
        else if (pwm > 0)
        {
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm);
        }
        else if(pwm == 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
        }
    }
}
//��������ת��PWM
void motorCallbackLB(double pwm)
{
    if (encodingMotorLB.GetReverse())
    {
        pwm = -pwm;
        if(pwm < 0)
        {
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000+pwm);
        }
        else if (pwm > 0)
        {
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
        }
        else if(pwm == 0)
        {
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        }
    }
    else
    {
        if(pwm < 0)
        {
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1000+pwm);
        }
        else if (pwm > 0)
        {
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
        }
        else if(pwm == 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
        }
    }

}
//��������ת��PWM
void motorCallbackRF(double pwm)
{
    if (encodingMotorRF.GetReverse())
    {
        pwm = -pwm;
        if (pwm < 0)
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000+pwm);
        }
        else if (pwm > 0)
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
        }
        else if(pwm == 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_RESET);
        }
    }

    else
    {
        if(pwm < 0)
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 1000+pwm);
        }
        else if (pwm > 0)
        {
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
        }
        else if(pwm == 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_6,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOG,GPIO_PIN_7,GPIO_PIN_RESET);
        }
    }
}
//��������ת��PWM
void motorCallbackRB(double pwm)
{
    if (encodingMotorRB.GetReverse())//����ת
    {
        pwm = -pwm;
        if(pwm < 0)
        {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000+pwm);
        }
        else if (pwm > 0)
        {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
        }
        else if(pwm == 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
        }
    }

    else{//������ת
        if(pwm < 0)
        {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 1000+pwm);
        }

        else if (pwm > 0)
        {
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
        }

        else if(pwm == 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
        }
    }

}


//PID��ʼ��
void RobotInit()
{
    double kp =  2.6, ki =  0.00128, kd =  0.0005; //LF
    double kp3 = 2.6, ki3 = 0.00128, kd3 = 0.0005; //LB
    double kpr = 2.6, kir = 0.00128, kdr = 0.0005; //RF
    double kp4 = 2.6, ki4 = 0.00128, kd4 = 0.0005; //RB

    encodingMotorRB.SetPID(kp4, ki4, kd4);
    encodingMotorRB.SetOutputRange(-1000, 1000);
    encodingMotorRB.SetSpeed(0);
    encodingMotorRB.SetCallback(motorCallbackRB);
    encodingMotorRB.SetReverse(true);

    encodingMotorLF.SetPID(kp, ki, kd);
    encodingMotorLF.SetOutputRange(-1000,   1000);
    encodingMotorLF.SetSpeed(0);
    encodingMotorLF.SetCallback(motorCallbackLF);
    encodingMotorLF.SetReverse(false);

    encodingMotorRF.SetPID(kpr, kir, kdr);
    encodingMotorRF.SetOutputRange(-1000, 1000);
    encodingMotorRF.SetSpeed(0);
    encodingMotorRF.SetCallback(motorCallbackRF);
    encodingMotorRF.SetReverse(true);

    encodingMotorLB.SetPID(kp3, ki3, kd3);
    encodingMotorLB.SetOutputRange(-1000, 1000);
    encodingMotorLB.SetSpeed(0);
    encodingMotorLB.SetCallback(motorCallbackLB);
    encodingMotorLB.SetReverse(false);

    twoWheelMotion.SetEncodingMotorRB(&encodingMotorRB);
    twoWheelMotion.SetEncodingMotorLF(&encodingMotorLF);
    twoWheelMotion.SetEncodingMotorRF(&encodingMotorRF);
    twoWheelMotion.SetEncodingMotorLB(&encodingMotorLB);

//    SpinMotion.SetEncodingMotorLF(&encodingMotorLF);
//    SpinMotion.SetEncodingMotorRF(&encodingMotorRF);
//    SpinMotion.SetEncodingMotorLB(&encodingMotorLB);
//    SpinMotion.SetEncodingMotorRB(&encodingMotorRB);
//    SpinMotion.LinkMotion(&twoWheelMotion);
//    forward_heading_fix = SpinMotion.GetNowHeading();
//    twoWheelMotion.SetSpinMotion(&SpinMotion);

}

void RoboTick()
{
    short pulse = 0;

    pulse = (short) __HAL_TIM_GET_COUNTER(&htim8);
    __HAL_TIM_SET_COUNTER(&htim8, 0);
    encodingMotorRB.AddPulse(pulse);    //����
    encodingMotorRB.Tick();

    pulse = (short) __HAL_TIM_GET_COUNTER(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    encodingMotorRF.AddPulse(pulse);    //����
    encodingMotorRF.Tick();

    pulse = (short) __HAL_TIM_GET_COUNTER(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    encodingMotorLB.AddPulse(pulse);    //����
    encodingMotorLB.Tick();

    pulse = (short) __HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    encodingMotorLF.AddPulse(pulse);    //����    1ms��ȡ�ı���������������Ϊ pid.input
    encodingMotorLF.Tick();

}

void RoboTest(){

}

void RobotMoveLineRadius(double lineSpeed, double Radius, bool left)
{
    twoWheelMotion.ClearSpeed();
    twoWheelMotion.AddLineSpeed(lineSpeed);
    twoWheelMotion.AddRadius(Radius, left);
    twoWheelMotion.CommitSpeed();
}


void RobotMoveSpeed(double lineSpeed)
{
    twoWheelMotion.ClearSpeed();
    twoWheelMotion.AddLineSpeed(lineSpeed);
    twoWheelMotion.CommitSpeed();
}

double RobotMoveForward(double speed, double ForwardDis) //ForwardDis��λΪcm
{
    ForwardDis = ForwardDis / 30 * 60000;
    twoWheelMotion.Move(speed, ForwardDis);
}

void RobotSpinNinety(bool left) {
    if (left)
        SpinMotion.spin_left();
    else
        SpinMotion.spin_right();
}

void RobotFixHeading()
{
    SpinMotion.spin_to(forward_heading_fix);
}

void refreshFixHeading()
{
    forward_heading_fix = SpinMotion.GetNowHeading();
}

void RobotClearOdometry()
{
    encodingMotorLF.ClearPulse();
    encodingMotorRF.ClearPulse();
    encodingMotorLB.ClearPulse();
    encodingMotorRB.ClearPulse();
}

uint32_t RobotGetOdometry()
{
    int pulse1 = encodingMotorLF.GetPulse();
    int pulse2 = encodingMotorRF.GetPulse();
    int pulse3 = encodingMotorLB.GetPulse();
    int pulse4 = encodingMotorRB.GetPulse();
    if (pulse1 < 0) pulse1 = -pulse1;
    if (pulse2 < 0) pulse2 = -pulse2;
    if (pulse3 < 0) pulse3 = -pulse3;
    if (pulse4 < 0) pulse4 = -pulse4;
    return (uint32_t)((pulse1 + pulse2 + pulse3 + pulse4) / 4.0);
}

//////////////////////////////////////////��PID��/////////////////////////////////////////////////////////////////////
void Transmit(){
    char bufferLF[32];
    sprintf(bufferLF, "%f \n", encodingMotorLF.pid.GetInput());
    HAL_UART_Transmit(&huart1, (uint8_t *)bufferLF, strlen(bufferLF),HAL_MAX_DELAY);
}


////////////////////////////////////////////��е���˶�/////////////////////////////////////////////////////////////////
Servo BusServo1(BusServo, 2 * PI / 3, 0, 1); //240�����߶�� ˳ʱ��Ƕȼ�С
Servo BusServo2(BusServo, 2 * PI / 3, 0, 2);
Servo BusServo3(BusServo, 2 * PI / 3, 0, 3);
Servo PwmServo1(PwmServo, 360, 0, 0); //360����̨���
Servo PwmServo2(PwmServo, 50, 0, 0); //180��צ�Ӷ��

void PwmServoCallback1(double pwm_now, double pwm_to_be) //360   PE6
{
    if (pwm_now <= pwm_to_be)
    {
        for (int i = pwm_now; i <= pwm_to_be; i++)
        {
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, i);  //PE6
            HAL_Delay(5);
        }
    } else
    {
        for (int i = pwm_now; i >= pwm_to_be; i--)
        {
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, i);  //PE6
            HAL_Delay(5);
        }
    }

}

void PwmServoCallback2(double pwm_now, double pwm_to_be) //180  PE5
{
    if (pwm_now <= pwm_to_be)
    {
        for (int i = pwm_now; i <= pwm_to_be; i++)
        {
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, i);  //PE5
            HAL_Delay(5);
        }
    } else
    {
        for (int i = pwm_now; i >= pwm_to_be; i--)
        {
            __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, i);  //PE5
            HAL_Delay(5);
        }
    }
}

void RobotInitServo(){
    PwmServo1.SetCallback(PwmServoCallback1);
    PwmServo2.SetCallback(PwmServoCallback2);
}

void RobotTestServo(){

    }


void RobotArmMiddle()
{
    BusServo2.MoveBusServoCmd(32.0/75*PI,1000);
    BusServo1.MoveBusServoCmd(23.0/30*PI,1000);
    BusServo3.MoveBusServoCmd(2.0/3*PI,1000);
    HAL_Delay(500); //�ȴ����߶�����
    PwmServo1.PwmCommitAngle(172.8,360);
    PwmServo2.PwmCommitAngle(90,180);
    HAL_Delay(800);

}

void RobotGrabRightUp()
{
    //׼��ʶ��Ķ�����
    BusServo2.MoveBusServoCmd(34.0/75*PI,1000);
    BusServo1.MoveBusServoCmd(47.0/60*PI,1000);
    BusServo3.MoveBusServoCmd(2.0/3*PI,1000);
    HAL_Delay(2000);
    PwmServo1.PwmCommitAngle(82.8,360);
    PwmServo2.PwmCommitAngle(90,180);
    HAL_Delay(500);

    rx_times_flag1 = 0;
    //��ʼʶ��
    while(rx_times_flag1<=800)
    {
        rx_times_flag2 = 0;
        HAL_UART_Receive_IT(&huart6, &single_byte, sizeof(single_byte));

        int none_flag = 0;
        HAL_Delay(3000); //ʶ������

        HAL_UART_Abort_IT(&huart6);
        if(!k210_msg_received_flag) none_flag = 1;
        if (k210_msg_received_flag) k210_msg_received_flag = 0;
        if (none_flag) break;

        int fruit_flag = 0;
        if (k210_msg_flag == 1)
        {
            k210_msg_flag = 0;
            if (k210_msg[0] == '5' && k210_msg[1] == '5' && k210_msg[2] == '5' && k210_msg[3] == '5' &&
                k210_msg[16] == '8' && k210_msg[15] == '8' && k210_msg[14] == '8' && k210_msg[13] == '8')
            {
                if (k210_msg[11] == '0' && (k210_msg[12] == '1' || k210_msg[12] == '2'))
                    fruit_flag = 1;
                if (k210_msg[11] == '0' && (k210_msg[12] == '4' || k210_msg[12] == '5'))
                    break;
            }
        }

        if (fruit_flag) {
            //����ˮ��ƫ����
            int centerX=0;
            int centerY=0;
            char center_x[3];
            char center_y[3];
            for (int i=0; i<3; i++)
            {
                center_x[i] = k210_msg[i+4];
                center_y[i] = k210_msg[i+8];
            }
            centerX = atoi(center_x);
            centerY = atoi(center_y);
            int Xoffset = centerX - 160;
            double AngleOffset = 0;
            AngleOffset = -Xoffset * 0.07;
            PwmServo1.PwmCommitAngle(PwmServo1.GetAngle() + AngleOffset, 360);
            HAL_Delay(100);

            int Yoffset = centerY - 120;
            AngleOffset = Yoffset * 0.12;
            AngleOffset = AngleOffset / 180 * PI;
            BusServo1.MoveBusServoCmd(BusServo1.GetAngle() - AngleOffset, 1000);
            HAL_Delay(1000);

            //ץȡˮ�� ץȡ�÷Ż�����
            PwmServo2.PwmCommitAngle(70, 180);
            HAL_Delay(500);
            BusServo2.MoveBusServoCmd(52.5/75*PI,1000); //470
            BusServo1.MoveBusServoCmd(59.0/75*PI - AngleOffset,1000); //590
            BusServo3.MoveBusServoCmd(2.0/3*PI,1000); //500
            HAL_Delay(1500);
            PwmServo2.PwmCommitAngle(22,180);//צ�ӱպ�
            HAL_Delay(1000);
            BusServo1.MoveBusServoCmd(BusServo1.GetAngle()+0.06*PI,1000); //̧��һ�� 620/��30
            HAL_Delay(1000);

            //��е�۷�ˮ��������
            BusServo2.MoveBusServoCmd(0.4*PI,1000); //300
            BusServo3.MoveBusServoCmd(2.0/3*PI,1000); //500
            HAL_Delay(1500);

            //�Ӽ����࣬��ץ��������С�����Ͳ���ץ��

            PwmServo1.PwmCommitAngle(0,360);//ת���
            HAL_Delay(1000);
            PwmServo2.PwmCommitAngle(90,180);//צ���ſ�
            HAL_Delay(1000);
            amount_of_fruits++;
            break;
        }
    }
}

void RobotGrabRightGround()
{
    //׼��ʶ��Ķ�����
    PwmServo1.PwmCommitAngle(82.8,360);
    PwmServo2.PwmCommitAngle(90,180);
//    HAL_Delay(1000);
    BusServo2.MoveBusServoCmd(13.0/15*PI,1000); //650
    BusServo1.MoveBusServoCmd(89.0/150*PI,1000); //455
    BusServo3.MoveBusServoCmd(44.0/150*PI,1000); //235
    HAL_Delay(2000);

    rx_times_flag1 = 0;
    //��ʼʶ��
    while(rx_times_flag1<=800)
    {
        rx_times_flag2 = 0;
        HAL_UART_Receive_IT(&huart6, &single_byte, sizeof(single_byte));

        int none_flag = 0;
        HAL_Delay(3000); //ʶ������

        HAL_UART_Abort_IT(&huart6);
        if(!k210_msg_received_flag) none_flag = 1;
        if (k210_msg_received_flag) k210_msg_received_flag = 0;
        if (none_flag) break;

        int fruit_flag = 0;
        if (k210_msg_flag == 1)
        {
            k210_msg_flag = 0;
            if (k210_msg[0] == '5' && k210_msg[1] == '5' && k210_msg[2] == '5' && k210_msg[3] == '5' &&
                k210_msg[16] == '8' && k210_msg[15] == '8' && k210_msg[14] == '8' && k210_msg[13] == '8')
            {
                if (k210_msg[11] == '0' && (k210_msg[12] == '1' || k210_msg[12] == '2'))
                    fruit_flag = 1;
                if (k210_msg[11] == '0' && (k210_msg[12] == '4' || k210_msg[12] == '5'))
                    break;
            }
        }

        if (fruit_flag) {
            //����ˮ��ƫ����
            int centerX=0;
            int centerY=0;
            char center_x[3];
            char center_y[3];
            for (int i=0; i<3; i++)
            {
                center_x[i] = k210_msg[i+4];
                center_y[i] = k210_msg[i+8];
            }
            centerX = atoi(center_x);
            centerY = atoi(center_y);
            int Xoffset = centerX - 160;
            double AngleOffset = 0;
            AngleOffset = -Xoffset * 0.07;
            PwmServo1.PwmCommitAngle(PwmServo1.GetAngle() + AngleOffset, 360);
            HAL_Delay(100);

            int Yoffset = centerY - 120;
            AngleOffset = Yoffset * 0.12;
            AngleOffset = AngleOffset / 180 * PI;
            BusServo2.MoveBusServoCmd(BusServo2.GetAngle() - AngleOffset, 1000);
            HAL_Delay(1000);

            //ץȡˮ�� ץȡ�÷Ż�����
            BusServo2.MoveBusServoCmd(32.0/30*PI - AngleOffset,1000); //775
            BusServo1.MoveBusServoCmd(40.0/75*PI,1000); //430
            BusServo3.MoveBusServoCmd(18.0/60*PI,1000); //275
            HAL_Delay(1500); //���߶����ɶ�����ʱ��
            PwmServo2.PwmCommitAngle(23,180);
            HAL_Delay(1000); //צ��ץȡˮ��ʱ��
            BusServo2.MoveBusServoCmd(0.4*PI,2000);
            BusServo1.MoveBusServoCmd(0.8*PI,1000); //600
            BusServo3.MoveBusServoCmd(2.0/3*PI,1000);
            HAL_Delay(1800);
            PwmServo1.PwmCommitAngle(0,360);
            HAL_Delay(1250);
            PwmServo2.PwmCommitAngle(90,180);
            HAL_Delay(1250);//��е�۷�ˮ��������
            amount_of_vegetables++;
            break;
        }
    }
}

void RobotGrabLeftUp()
{
    //׼��ʶ��Ķ�����
    BusServo2.MoveBusServoCmd(35.0/75*PI,1000);
    BusServo1.MoveBusServoCmd(47.0/60*PI,1000);
    BusServo3.MoveBusServoCmd(2.0/3*PI,1000);
    HAL_Delay(2000);
    PwmServo1.PwmCommitAngle(260.8,360);
    PwmServo2.PwmCommitAngle(90,180);
    HAL_Delay(500);

    rx_times_flag1 = 0;
    //��ʼʶ��
    while(rx_times_flag1<=800)
    {
        rx_times_flag2 = 0;
        HAL_UART_Receive_IT(&huart6, &single_byte, sizeof(single_byte));

        int none_flag = 0;
        HAL_Delay(5000); //ʶ������

        HAL_UART_Abort_IT(&huart6);
        if(!k210_msg_received_flag) none_flag = 1;
        if (k210_msg_received_flag) k210_msg_received_flag = 0;
        if (none_flag) break;

        int fruit_flag = 0;
        if (k210_msg_flag == 1)
        {
            k210_msg_flag = 0;
            if (k210_msg[0] == '5' && k210_msg[1] == '5' && k210_msg[2] == '5' && k210_msg[3] == '5' &&
                k210_msg[16] == '8' && k210_msg[15] == '8' && k210_msg[14] == '8' && k210_msg[13] == '8')
            {
                if (k210_msg[11] == '0' && (k210_msg[12] == '1' || k210_msg[12] == '2'))
                    fruit_flag = 1;
                if (k210_msg[11] == '0' && (k210_msg[12] == '4' || k210_msg[12] == '5'))
                    break;
            }

        }

        if (fruit_flag) {
            //����ˮ��ƫ����
            int centerX=0;
            int centerY=0;
            char center_x[3];
            char center_y[3];
            for (int i=0; i<3; i++)
            {
                center_x[i] = k210_msg[i+4];
                center_y[i] = k210_msg[i+8];
            }
            centerX = atoi(center_x);
            centerY = atoi(center_y);
            int Xoffset = centerX - 160;
            double AngleOffset = 0;
            AngleOffset = -Xoffset * 0.07;
            PwmServo1.PwmCommitAngle(PwmServo1.GetAngle() + AngleOffset, 360);
            HAL_Delay(100);

            int Yoffset = centerY - 120;
            AngleOffset = Yoffset * 0.12;
            AngleOffset = AngleOffset / 180 * PI;
            BusServo1.MoveBusServoCmd(BusServo1.GetAngle() - AngleOffset, 1000);
            HAL_Delay(1000);

            //ץȡˮ�� ץȡ�÷Ż�����
            PwmServo2.PwmCommitAngle(70, 180);
            HAL_Delay(500);
            BusServo2.MoveBusServoCmd(52.5/75*PI,1000); //470
            BusServo1.MoveBusServoCmd(60.0/75*PI - AngleOffset,1000); //590
            BusServo3.MoveBusServoCmd(2.0/3*PI,1000); //500
            HAL_Delay(1500);
            PwmServo2.PwmCommitAngle(23,180);
            HAL_Delay(1250);
            BusServo1.MoveBusServoCmd(BusServo1.GetAngle()+0.06*PI,1000); //̧��һ�� 620/+30
            HAL_Delay(1000);
            BusServo2.MoveBusServoCmd(0.4*PI,1000); //300
            BusServo3.MoveBusServoCmd(2.0/3*PI,1000); //500
            HAL_Delay(1250);
            PwmServo1.PwmCommitAngle(352.8,360);
            HAL_Delay(1250);
            PwmServo2.PwmCommitAngle(90,180);
            HAL_Delay(1250);//��е�۷�ˮ��������
            amount_of_fruits++;
            break;
        }
    }
}

void RobotGrabLeftGround()
{
    //׼��ʶ��Ķ�����
    PwmServo1.PwmCommitAngle(262.8,360);
    PwmServo2.PwmCommitAngle(90,180);
//    HAL_Delay(500);
    BusServo2.MoveBusServoCmd(13.0/15*PI,1000); //650
    BusServo1.MoveBusServoCmd(89.0/150*PI,1000); //455
    BusServo3.MoveBusServoCmd(44.0/150*PI,1000); //235
    HAL_Delay(2000);

    rx_times_flag1 = 0;
    //��ʼʶ��
    while(rx_times_flag1<=800)
    {
        rx_times_flag2 = 0;
        HAL_UART_Receive_IT(&huart6, &single_byte, sizeof(single_byte));

        HAL_Delay(3000);
        int none_flag = 0;

        HAL_UART_Abort_IT(&huart6);
        if(!k210_msg_received_flag) none_flag = 1;
        if (k210_msg_received_flag) k210_msg_received_flag = 0;
        if (none_flag) break;

        int fruit_flag = 0;
        if (k210_msg_flag == 1)
        {
            k210_msg_flag = 0;
            if (k210_msg[0] == '5' && k210_msg[1] == '5' && k210_msg[2] == '5' && k210_msg[3] == '5' &&
                k210_msg[16] == '8' && k210_msg[15] == '8' && k210_msg[14] == '8' && k210_msg[13] == '8')
            {
                if (k210_msg[11] == '0' && (k210_msg[12] == '1' || k210_msg[12] == '2'))
                    fruit_flag = 1;
                if (k210_msg[11] == '0' && (k210_msg[12] == '4' || k210_msg[12] == '5'))
                    break;
            }

        }

        if (fruit_flag) {
            //����ˮ��ƫ����
            int centerX=0;
            int centerY=0;
            char center_x[3];
            char center_y[3];
            for (int i=0; i<3; i++)
            {
                center_x[i] = k210_msg[i+4];
                center_y[i] = k210_msg[i+8];
            }
            centerX = atoi(center_x);
            centerY = atoi(center_y);
            int Xoffset = centerX - 160;
            double AngleOffset = 0;
            AngleOffset = -Xoffset * 0.07;
            PwmServo1.PwmCommitAngle(PwmServo1.GetAngle() + AngleOffset, 360);
            HAL_Delay(100);

            int Yoffset = centerY - 120;
            AngleOffset = Yoffset * 0.12;
            AngleOffset = AngleOffset / 180 * PI;
            BusServo2.MoveBusServoCmd(BusServo2.GetAngle() - AngleOffset, 1000);
            HAL_Delay(1000);

            //ץȡˮ�� ץȡ�÷Ż�����
            BusServo2.MoveBusServoCmd(32.0/30*PI - AngleOffset,1000); //775
            BusServo1.MoveBusServoCmd(40.0/75*PI,1000); //430
            BusServo3.MoveBusServoCmd(18.0/60*PI,1000); //275
            HAL_Delay(1500); //���߶����ɶ�����ʱ��
            PwmServo2.PwmCommitAngle(22,180);
            HAL_Delay(1250); //צ��ץȡˮ��ʱ��
            BusServo2.MoveBusServoCmd(0.4*PI,2000);
            BusServo1.MoveBusServoCmd(0.8*PI,1000); //600
            BusServo3.MoveBusServoCmd(2.0/3*PI,1000);
            HAL_Delay(2100);
            PwmServo1.PwmCommitAngle(352.8,360);
            HAL_Delay(1250);
            PwmServo2.PwmCommitAngle(90,180);
            HAL_Delay(1250);//��е�۷�ˮ��������
            amount_of_vegetables++;
            break;
        }
    }
}

void RobotCGrabLeft()
{
    //׼��ʶ��Ķ�����
    PwmServo1.PwmCommitAngle(262.8,360);
    PwmServo2.PwmCommitAngle(90,180);
    HAL_Delay(2000);
    BusServo2.MoveBusServoCmd(0.8*PI,1000); //600
    BusServo1.MoveBusServoCmd(8.0/15*PI,1000); //400
    BusServo3.MoveBusServoCmd(0.2*PI,1000); //150
    HAL_Delay(2000);

    rx_times_flag1 = 0;
    //��ʼʶ��
    while(rx_times_flag1<=800)
    {
        rx_times_flag2 = 0;
        HAL_UART_Receive_IT(&huart6, &single_byte, sizeof(single_byte));

        HAL_Delay(3000);
        int none_flag = 0;

        HAL_UART_Abort_IT(&huart6);
        if(!k210_msg_received_flag) none_flag = 1;
        if (k210_msg_received_flag) k210_msg_received_flag = 0;
        if (none_flag) break;//û�в��ҵ����˶�

        int fruit_flag = 0;
        if (k210_msg_flag == 1)
        {
            k210_msg_flag = 0;
            if (k210_msg[0] == '5' && k210_msg[1] == '5' && k210_msg[2] == '5' && k210_msg[3] == '5' &&
                k210_msg[16] == '8' && k210_msg[15] == '8' && k210_msg[14] == '8' && k210_msg[13] == '8')
            {
                if (k210_msg[11] == '0' && (k210_msg[12] == '1' || k210_msg[12] == '2'))
                    fruit_flag = 1;
                if (k210_msg[11] == '0' && (k210_msg[12] == '4' || k210_msg[12] == '5'))
                    break;//���ҵ���ɫ���˶�
            }
        }

        if (fruit_flag) {
            //����ˮ��ƫ����
            int centerX=0;
            int centerY=0;
            char center_x[3];
            char center_y[3];
            for (int i=0; i<3; i++)
            {
                center_x[i] = k210_msg[i+4];
                center_y[i] = k210_msg[i+8];
            }
            centerX = atoi(center_x);
            centerY = atoi(center_y);
            int Xoffset = centerX - 160;
            double AngleOffset = 0;
            AngleOffset = -Xoffset * 0.07;
            PwmServo1.PwmCommitAngle(PwmServo1.GetAngle() + AngleOffset, 360);
            HAL_Delay(100);

            int Yoffset = centerY - 120;
            AngleOffset = Yoffset * 0.04;
            AngleOffset = AngleOffset / 180 * PI;
            BusServo2.MoveBusServoCmd(BusServo2.GetAngle() - AngleOffset, 1000);
            HAL_Delay(1000);

            //ץȡˮ�� ץȡ�÷Ż�����
            BusServo2.MoveBusServoCmd(13.0/15*PI - AngleOffset,1000); //650
            BusServo1.MoveBusServoCmd(11.0/30*PI,1000); //275
            BusServo3.MoveBusServoCmd(7.0/15*PI,1000); //350
            HAL_Delay(1500); //���߶����ɶ�����ʱ��
            PwmServo2.PwmCommitAngle(23,180);
            HAL_Delay(1250); //צ��ץȡˮ��ʱ��
            BusServo2.MoveBusServoCmd(0.4*PI,2000);
            BusServo1.MoveBusServoCmd(0.8*PI,1000); //600
            BusServo3.MoveBusServoCmd(2.0/3*PI,1000);
            HAL_Delay(2100);
            PwmServo1.PwmCommitAngle(352.8,360);
            HAL_Delay(1250);
            PwmServo2.PwmCommitAngle(90,180);
            HAL_Delay(1250);//��е�۷�ˮ��������
            amount_of_vegetables++;
            break;
        }
    }
}

void RobotCGrabRight()
{
    //׼��ʶ��Ķ�����
    PwmServo1.PwmCommitAngle(82.8,360);
    PwmServo2.PwmCommitAngle(90,180);
    HAL_Delay(1500);
    BusServo2.MoveBusServoCmd(0.8*PI,1000); //600
    BusServo1.MoveBusServoCmd(8.0/15*PI,1000); //400
    BusServo3.MoveBusServoCmd(0.2*PI,1000); //150
    HAL_Delay(1500);


    rx_times_flag1 = 0;
    //��ʼʶ��һ���ռ�800�����ݣ�
    while(rx_times_flag1<=800)
    {
        rx_times_flag2 = 0;
        HAL_UART_Receive_IT(&huart6, &single_byte, sizeof(single_byte));

        HAL_Delay(3000);
        int none_flag = 0;

        HAL_UART_Abort_IT(&huart6);
        if(!k210_msg_received_flag) none_flag = 1;
        if (k210_msg_received_flag) k210_msg_received_flag = 0;
        if (none_flag) break;

        int fruit_flag = 0;
        if (k210_msg_flag == 1)
        {
            k210_msg_flag = 0;
            if (k210_msg[0] == '5' && k210_msg[1] == '5' && k210_msg[2] == '5' && k210_msg[3] == '5' &&
                k210_msg[16] == '8' && k210_msg[15] == '8' && k210_msg[14] == '8' && k210_msg[13] == '8')
            {
                if (k210_msg[11] == '0' && (k210_msg[12] == '1' || k210_msg[12] == '2'))
                    fruit_flag = 1;
                if (k210_msg[11] == '0' && (k210_msg[12] == '4' || k210_msg[12] == '5'))
                    break;
            }
        }

        if (fruit_flag) {
            //����ˮ��ƫ����
            int centerX=0;
            int centerY=0;
            char center_x[3];
            char center_y[3];
            for (int i=0; i<3; i++)
            {
                center_x[i] = k210_msg[i+4];
                center_y[i] = k210_msg[i+8];
            }
            centerX = atoi(center_x);
            centerY = atoi(center_y);
            int Xoffset = centerX - 160;
            double AngleOffset = 0;
            AngleOffset = -Xoffset * 0.07;
            PwmServo1.PwmCommitAngle(PwmServo1.GetAngle() + AngleOffset, 360);
            HAL_Delay(100);

            int Yoffset = centerY - 120;
            AngleOffset = Yoffset * 0.04;
            AngleOffset = AngleOffset / 180 * PI;
            BusServo2.MoveBusServoCmd(BusServo2.GetAngle() - AngleOffset, 1000);
            HAL_Delay(1000);

            //ץȡˮ�� ץȡ�÷Ż�����
            BusServo2.MoveBusServoCmd(13.0/15*PI - AngleOffset,1000); //650
            BusServo1.MoveBusServoCmd(11.0/30*PI,1000); //275
            BusServo3.MoveBusServoCmd(7.0/15*PI,1000); //350
            HAL_Delay(1500); //���߶����ɶ�����ʱ��
            PwmServo2.PwmCommitAngle(23,180);
            HAL_Delay(1250); //צ��ץȡˮ��ʱ��
            BusServo2.MoveBusServoCmd(0.4*PI,2000);
            BusServo1.MoveBusServoCmd(0.8*PI,1000); //600
            BusServo3.MoveBusServoCmd(2.0/3*PI,1000);
            HAL_Delay(2100);
            PwmServo1.PwmCommitAngle(0,360);
            HAL_Delay(1250);
            PwmServo2.PwmCommitAngle(90,180);
            HAL_Delay(1250);//��е�۷�ˮ��������
            amount_of_vegetables++;
            break;
        }
    }
}

//K210�Ľ����ж�  �ߵ��Ľ����ж�
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        k210_msg_received_flag = 1;
        k210_msg[k210_msg_index] = single_byte;
        k210_msg_index++;
        if (k210_msg_index >= 18 || single_byte == '\n') {
            k210_msg_index = 0;
            k210_msg_flag = 1;
        }

        rx_times_flag1++;
        rx_times_flag2++;
        HAL_UART_Receive_IT(&huart6, &single_byte, sizeof(single_byte));
    }

    if (huart->Instance == USART2) {
        RxCplt_flag = 1;
        for (int i = 0; i < 1024; i++) {
            ahrs_data_buffer1[i] = ahrs_data_buffer[i];
        }
    }
}

////////////////////////////////////ȫ���˶�/////////////////////////////////////////////////////
//��ʼ����������
//Music:ѡ�񱳾����֡�0:�ޱ������֣�1~15��ѡ�񱳾�����
void RobotBeginVoice(uint8_t Music, uint8_t *HZdata)
{
    /****************��Ҫ���͵��ı�**********************************/
    unsigned  char  Frame_Info[50];
    unsigned  char  HZ_Length;
    unsigned  char  ecc  = 0;  			//����У���ֽ�
    unsigned  int i = 0;
    HZ_Length = strlen((char*)HZdata); 			//��Ҫ�����ı��ĳ���

    /*****************֡�̶�������Ϣ**************************************/
    Frame_Info[0] = 0xFD ; 			//����֡ͷFD
    Frame_Info[1] = 0x00 ; 			//�������������ȵĸ��ֽ�
    Frame_Info[2] = HZ_Length + 3; 		//�������������ȵĵ��ֽ�
    Frame_Info[3] = 0x01 ; 			//���������֣��ϳɲ�������
    Frame_Info[4] = 0x01 | Music << 4 ; //����������������������趨

    /*******************У�������***************************************/
    for(i = 0; i < 5; i++)   				//���η��͹���õ�5��֡ͷ�ֽ�
    {
        ecc = ecc ^ (Frame_Info[i]);		//�Է��͵��ֽڽ������У��
    }

    for(i = 0; i < HZ_Length; i++)   		//���η��ʹ��ϳɵ��ı�����
    {
        ecc = ecc ^ (HZdata[i]); 				//�Է��͵��ֽڽ������У��
    }
    /*******************����֡��Ϣ***************************************/
    memcpy(&Frame_Info[5], HZdata, HZ_Length);


    Frame_Info[5 + HZ_Length] = ecc;
    HAL_UART_Transmit(&huart5, (uint8_t*)Frame_Info, 5 + HZ_Length + 1, HAL_MAX_DELAY);
}


void RobotReceivedFruitVoice() {

}

void RoboAllMove() {
    ///////////////////////////////////AAAAAAAAAAAAAAAAAAAAAAAAAAA����//////////////////////////////////////////////
    RobotMoveForward(30, 48);//ǰ�ֵ�������8cm
    RobotArmMiddle();
    RobotGrabRightGround();
    RobotArmMiddle();
    RobotGrabLeftUp();
    RobotArmMiddle();

    RobotMoveForward(30, 100);
    RobotArmMiddle();
    RobotGrabRightUp();
    RobotArmMiddle();
    RobotGrabLeftGround();
    RobotArmMiddle();

    RobotMoveForward(30, 100);
    RobotArmMiddle();
    RobotGrabRightGround();
    RobotArmMiddle();
    RobotGrabLeftUp();
//    RobotArmMiddle();

    RobotMoveForward(30, 60);
    RobotSpinNinety(false);
    //���½Ƕ�
    ///////////////////////////////////BBBBBBBBBBBBBBBBBBBBBBBBBBB����/////////////////////////////////////////////////

    RobotMoveForward(50, 100);
    RobotSpinNinety(false);

    RobotMoveForward(50, 60);
    RobotArmMiddle();
    RobotGrabRightGround();
    RobotArmMiddle();
    RobotGrabLeftUp();
    RobotArmMiddle();

    RobotMoveForward(50, 100);
    RobotArmMiddle();
    RobotGrabRightUp();
    RobotArmMiddle();
    RobotGrabLeftGround();
    RobotArmMiddle();

    RobotMoveForward(50, 100);
    RobotArmMiddle();
    RobotGrabRightGround();
    RobotArmMiddle();
    RobotGrabLeftUp();
    RobotArmMiddle();
    /////////////////////////////////CCCCCCCCCCCCCCCCCCCCCCCCC��////////////////////////////////////////////////////////

//    HAL_Delay(3000);
//    RobotReceivedFruitVoice();

}

