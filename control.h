#ifndef _CONTROL_H_
#define _CONTROL_H_
typedef struct PID{float P,pout,I,iout,D,dout,OUT;}PID;
extern int16 Car_State;
extern  PID PID_ANGLE,PID_SPEED,PID_TURN,PID_TURN_Lie;
//extern int16 Inductor_ADC[8];
extern float Angle,Angle_Speed,Car_Angle,Turn_Speed;
extern int16  Gyro_X_Offset,Gyro_Y_Offset,Acc_Offset; //传感器偏差
extern int Speed_Filter_Times;
extern int SpeedCount;
extern float CarSpeed,ControlSpeed;
extern float SetSpeed;
//extern uint8 Set_Angle;
//extern float AverageSpeed;
extern float Distance;
extern float SpeedControlOutOld,SpeedControlOutNew;
extern float SpeedControlIntegral;
extern float LeftMotorOut,RightMotorOut;   //电机输出量
//模糊化相关
//extern float  Delta_P;
//extern float  Delta_D;
//extern float  Fuzzy_Kp;
//extern float  Fuzzy_Kd;
//方向控制相关
extern int    DirectionCount;
extern float  Delt_error,Middle_Err;
extern float  Turn_Speed,Turn_Out,Turn_Angle_Integral;

extern uint8 Protect,ForceStop,Starting,Stop,CarStopedJustNow;

void Get_Attitude();
void Get_Speed();
void Strong_Turn();
void Angle_Calculate();
void Angle_Control();
void Moto_Out();
void Speed_Control();
void Speed_Control_Output();
float  Middle_Err_Filter(float middle_err);  
float  Turn_Out_Filter(float turn_out);
void Push_And_Pull(float *buff,int len,float newdata);
float Slope_Calculate(uint8 begin,uint8 end,float *p);
void GET_MIN();
void GET_MAX();
void roadturncal();
#endif
