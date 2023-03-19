#include "move.h"

PidParms WheelPidParms, YawPidParms;
WheelPidVars WheelPid[4];
YawPidVars YawPid;
float SetRpm[4];
int WheelCounts[4];
StateInfo AxisData;
float YawDifference;

void InitPid()
{
    for (int i = 0; i < 4; i++)
    {
        WheelPid[i].rpm = 0;
        WheelPid[i].pwm = 0;
        WheelPid[i].Err = 0;
        WheelPid[i].dErr = 0;
        WheelPid[i].ErrSum = 0;
        SetRpm[i] = 0;
        WheelCounts[i] = 0;
    }
    YawPid.currentyaw = 0;
    YawPid.goalyaw = 0;
    YawPid.goalv = 0;
    YawPid.Err = 0;
    YawPid.dErr = 0;
    YawPid.ErrSum = 0;
    YawDifference = 0;
}

void WheelPidCalculate()
{
    for (int i = 0; i < 4; i++)
    {
        if (WheelCounts[i] > 65535 - Maxrpm)
        {
            WheelCounts[i] -= 65535;
        }
        WheelPid[i].rpm = WheelCounts[i] * pidFeq / CountPerRound;
        WheelPid[i].dErr = dErrLastRaio * (SetRpm[i] - WheelPid[i].rpm - WheelPid[i].Err) + (1 - dErrLastRaio) * WheelPid[i].dErr;
        WheelPid[i].Err = SetRpm[i] - WheelPid[i].rpm;

        if (WheelPid[i].ErrSum > IntegralLimit)
        {
            if (WheelPid[i].Err < 0)
            {
                WheelPid[i].ErrSum += WheelPid[i].Err;
            }
        }
        else if (WheelPid[i].ErrSum < -IntegralLimit)
        {
            if (WheelPid[i].Err > 0)
            {
                WheelPid[i].ErrSum += WheelPid[i].Err;
            }
        }
        else{
            WheelPid[i].ErrSum += WheelPid[i].Err;
        }

        float output = WheelPidParms.kp * WheelPid[i].Err + WheelPidParms.kd * WheelPid[i].dErr + WheelPidParms.ki * WheelPid[i].ErrSum;
        if (output > pidLimit)
        {
            output = pidLimit;
        }
        else if (output < -pidLimit)
        {
            output = -pidLimit;
        }
        WheelPid[i].pwm = pwmLastRatio * output + (1 - pwmLastRatio) * WheelPid[i].pwm;
    }
}

void MoveBasic(float onPitchAxis, float onRollAxis, float rotateYawAxis)
{
    SetRpm[0] = (onRollAxis + onPitchAxis - RotateSpeedGain * rotateYawAxis);
    SetRpm[1] = (onRollAxis - onPitchAxis + RotateSpeedGain * rotateYawAxis);
    SetRpm[2] = (onRollAxis - onPitchAxis - RotateSpeedGain * rotateYawAxis);
    SetRpm[3] = -(onRollAxis + onPitchAxis + RotateSpeedGain * rotateYawAxis);
}

void SetStateInfo(float accx, float accy, float accz, float velox, float veloy, float veloz, float pitch, float yaw, float roll)
{
    AxisData.accx = accx;
    AxisData.accx = accy;
    AxisData.accx = accz;
    AxisData.velox = velox;
    AxisData.veloy = veloy;
    AxisData.veloz = veloz;
    AxisData.pitch = pitch;
    AxisData.yaw = yaw;
    AxisData.roll = roll;
}

void YawCalibration(float vx, float vy)
{
    YawDifference = atan2f(vx, vy) - AxisData.yaw;
}

void YawPidCalculate()
{
    YawPid.currentyaw = AxisData.yaw + YawDifference;
    YawPid.Err = YawPid.goalyaw - YawPid.currentyaw;
    YawPid.dErr = AxisData.accz;
    if (YawPid.ErrSum > IntegralLimit)
        {
            if (YawPid.Err < 0)
            {
                YawPid.ErrSum += YawPid.Err;
            }
        }
        else if (YawPid.ErrSum < -IntegralLimit)
        {
            if (YawPid.Err > 0)
            {
                YawPid.ErrSum += YawPid.Err;
            }
        }
        else{
            YawPid.ErrSum += YawPid.Err;
        }
    YawPid.dYawOutput = YawPidParms.kp * YawPid.Err + YawPidParms.kd * YawPid.dErr + YawPidParms.ki * YawPid.ErrSum;
    //当pid输出值为正时，目标方向将顺时针偏转
    MoveBasic(YawPid.goalv * sin(YawPid.goalyaw + YawPid.dYawOutput), YawPid.goalv * cos(YawPid.goalyaw + YawPid.dYawOutput), 0);
}

void MoveByAbs(float v, float degree)
{
    YawPid.goalv = v;
    YawPid.goalyaw = degree;
}