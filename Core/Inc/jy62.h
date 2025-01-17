/*-------jy62.h-----------------------------*/
/*
说明：
1.把.h/.c文件放到Inc和Src里面，然后在Makefile里面添加.c文件的路径
2.开启对应串口DMA中断以及串口全局中断，在中断回调函数里面添加以下代码：

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
 {
   
    if(huart == (jy62选择的串口))
    {
      jy62MessageRecord();
    }
}

3.在main.c中User Code 2 处使能中断

void jy62_Init(UART_HandleTypeDef* huart); 

4.之后可在主函数中调用下面的接口函数来获得相应值
*/

#ifndef JY62_H
#define JY62_H
#include "stm32f1xx_hal.h"
#define JY62_MESSAGE_LENGTH 11
#define g 9.8f   //定义重力加速度值

struct Acce
{
    float accelerate_x;
    float accelerate_y;
    float accelerate_z;
};

struct Velo
{
    float velocity_x;
    float velocity_y;
    float velocity_z;
};

struct Angl 
{
    float roll;
    float pitch;
    float yaw;
};

struct Temp
{
    float temperature;
};

/**************接口*************************/
void jy62_Init(UART_HandleTypeDef* huart);    //初始化
void jy62_MessageRecord(void);              //实时记录信息，在每次接收完成后更新数据，重新开启中断

void jy62_SetBaud(int Baud);     //设置波特率，可选择115200或9600
void jy62_SetHorizontal(void);  //设置为水平放置模式
void jy62_SetVertical(void);  //设置为垂直放置模式
void jy62_InitAngle(void);    //初始化z轴角度为0
void jy62_Calibrate(void);    //校准加速度计
void jy62_SleepOrAwake(void);   //设置休眠/唤醒

float jy62_GetVeloX(void);       //获得角速度
float jy62_GetVeloY(void);
float jy62_GetVeloZ(void);

float jy62_GetAccX(void);        //获得加速度
float jy62_GetAccY(void);
float jy62_GetAccZ(void);

float jy62_GetRoll(void);     //绕x轴旋转角度（横滚角）
float jy62_GetPitch(void);    //绕y轴旋转角度（俯仰角）
float jy62_GetYaw(void);     //绕z轴旋转角度（偏航角）
float jy62_GetTemperature(void);

/****************************************************/
void Decode();

#endif
