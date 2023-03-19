#ifndef MOVE_H
#define MOVE_H

#include "stdint.h"
#include "math.h"

//每转一圈编码盘计数
#define CountPerRound 13
//pid所在中断的频率（单位Hz）
#define pidFeq 20
//限制pid输出值的绝对值
#define pidLimit 1000
//控制dErr的平滑程度，范围0~1，当等于1时只采用最新计算出的dErr，否则考虑与上一次的dErr进行加权平均，一般取值0.6~1
#define dErrLastRaio 0.9
//控制pwm输出的平滑程度，范围0~1，等于1时计算pwm只考虑当次的pid输出，否则考虑与上一次的pwm加权平均后再输出，一般取值0.6~1
#define pwmLastRatio 0.9
//限制积分变量的绝对值
#define IntegralLimit 800
//最大转速绝对值
#define Maxrpm 2000
//转向速度增益
#define RotateSpeedGain 0.5

//此结构体存储pid系数
typedef struct
{
    float kp;
    float kd;
    float ki;
}PidParms;

//此结构体存储轮子转速pid变量
typedef struct
{
    float rpm;
    float pwm;
    float Err;
    float dErr;
    float ErrSum;
}WheelPidVars;

//此结构体存储小车方向pid变量
typedef struct 
{
    float currentyaw;
    float goalyaw;
    float goalv;
    float dYawOutput;
    float Err;
    float dErr;
    float ErrSum;
}YawPidVars;

//此结构体存储方向和加速度等数据
typedef struct
{
    float accx, accy, accz;
    float velox, veloy, veloz;
    float pitch, yaw, roll;
}StateInfo;

//初始化
void InitPid();

//计算得出轮子的pwm值
void WheelPidCalculate();

//基本移动函数，分别控制前后、左右、旋转分量来控制移动
void MoveBasic(float onPitchAxis, float onRollAxis, float rotateYawAxis);

void SetStateInfo(float accx, float accy, float accz, float velox, float veloy, float veloz, float pitch, float yaw, float roll);

//输入当前速度在X, Y上的分量来校准方位角，使得当小车朝向Y正方向移动时方位角为0，当小车朝向X正方向移动时方位角为90
void YawCalibration(float x, float y);

//计算得出轮子的pwm值
void YawPidCalculate();

//移动函数，输入速度以及角度（与Y轴正方向所成夹角）
void MoveByAbs(float v, float degree);

#endif