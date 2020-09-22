#include "headfile.h"
#define TURNPWM_MAX  0.52   

//int16 Inductor_ADC[8];
// 转向AD控制参数
uint16   AD_val_1=0;
uint16   AD_val_2=0;
uint16   AD_val_3=0;
uint16   AD_val_4=0;
uint16   AD_val_5=0;
uint16   AD_val_6=0;
uint16   dis_AD_val_1,dis_AD_val_2,dis_AD_val_3,dis_AD_val_4,dis_AD_val_5,dis_AD_val_6 ;
uint16   disgy_AD_val_1,disgy_AD_val_2,disgy_AD_val_3,disgy_AD_val_4,disgy_AD_val_5,disgy_AD_val_6 ;
uint16   AD_val_1_min=0;
uint16   AD_val_2_min=0;
uint16   AD_val_3_min=0;
uint16   AD_val_6_min=0;

uint16   AD_val_1_max=3400;
uint16   AD_val_2_max=3400;
uint16   AD_val_3_max=3400;
uint16   AD_val_6_max=0;
int16    Car_State;
//角度类变量
int16  Gyro_X,Gyro_Y,Gyro_Z;
int16  Acc_X,Acc_Y,Acc_Z;
int16  Acc_Max=8192;
int16  Acc_Min=-8192;
int16  Acc_Offset;
float Angle,Angle_Speed,Car_Angle=0;
int16  Gyro_X_Offset,Gyro_Y_Offset;
int16  disangle;
//速度类变量
float SpeedControlOutNew=0.2;
float SpeedControlOutOld;
float SpeedControlIntegral=0;
float SetSpeed_ZL;
int   SpeedCount;
//int   Speed_Filter_Times=50;    //速度平滑输出
float CarSpeed=0,ControlSpeed=0,SetSpeed=0,Set_Speed;
//方向类变量
float DirectionControlOutNew;
float DirectionControlOutOld;
float Turn_Speed=0;
uint8   Dir_last;
//float Delt_error;
float Turn_Out;
//float Turn_Angle_Integral;
float DuoP,DuoD;
float  DirectionErr[9]={0},Error_Delta,Error_Delta_Deceleration;
float DIR_ERROR;
uint8 Calculate_Length,stringflag;
//int16 dischargecnt;
int16 stringcnt;
//模糊化系数
//float  Delta_P;
//float  Delta_D;
//float  Fuzzy_Kp;
//float  Fuzzy_Kd;

extern float Angle_OFF;

//PID控制类变量
PID PID_ANGLE,PID_SPEED,PID_TURN,PID_TURN_Lie;

float  LeftMotorOut,RightMotorOut;   //电机输出量
float  L_DeadValue=0,R_DeadValue=0;                
uint8   Stop;
uint8 Encoder_Disable=0;
extern uint8 workmode;
//
void Get_Attitude()
{
  Acc_Z =Get_Z_Acc();
  Gyro_X= Get_X_Gyro();
  Gyro_Y= Get_Y_Gyro();            
}
//******Kalman滤波******//
//-------------------------------------------------------
static  float Q_angle=0.001, Q_gyro=0.001, R_angle=5, dt=0.004;
	//Q增大，动态响应增大
static float Pk[2][2] = { {1, 0}, {0, 1 }};
	
static float Pdot[4] ={0,0,0,0};

static float q_bias=0, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
//-------------------------------------------------------
void Kalman_Filter(float angle_m,float gyro_m)			
{
	Car_Angle+=(gyro_m-q_bias) * dt; ///预测值
	Pdot[0]=Q_angle - Pk[0][1] - Pk[1][0];
	Pdot[1]=- Pk[1][1];
	Pdot[2]=- Pk[1][1];
	Pdot[3]=Q_gyro;
	
	Pk[0][0] += Pdot[0] * dt;
	Pk[0][1] += Pdot[1] * dt;
	Pk[1][0] += Pdot[2] * dt;
	Pk[1][1] += Pdot[3] * dt;
	
	angle_err = angle_m -Car_Angle;///测量值-预测值
	
	PCt_0 =  Pk[0][0];
	PCt_1 =  Pk[1][0];
	
	E = R_angle + PCt_0;
	
	K_0 = PCt_0 / E; ///卡尔曼增益
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = Pk[0][1];

	Pk[0][0] -= K_0 * t_0;
	Pk[0][1] -= K_0 * t_1;
	Pk[1][0] -= K_1 * t_0;
	Pk[1][1] -= K_1 * t_1;
	
	Car_Angle+= K_0 * angle_err; ///最优角度=预测值+卡尔曼增益*(测量值-预测值)
	q_bias	+= K_1 * angle_err;
	Angle_Speed = gyro_m-q_bias;
}


//角度计算与滤波
void Angle_Calculate()
{ float temp;    
  float res=0;
  float ratioa=0.048,ratiot=0.01;;
  
  Angle =(Acc_Z-Acc_Offset)*180.0/(Acc_Max-Acc_Min) - 20; 
  disangle = Angle * 10;
  Turn_Speed=(Gyro_X-Gyro_X_Offset)*ratiot;
  Angle_Speed=(Gyro_Y-Gyro_Y_Offset) * ratioa;
  Kalman_Filter(Angle,Angle_Speed);            //调用卡尔曼滤波函数
}

//角度控制函数
void Angle_Control()  
{
  PID_ANGLE.pout=PID_ANGLE.P*Car_Angle;
  PID_ANGLE.dout=PID_ANGLE.D*Angle_Speed;
  if(ABS(Angle_Speed)>30&&ABS(Angle_Speed)<80)
  {
       PID_ANGLE.dout*=(1+(ABS(Angle_Speed)-30.0)/30.0);
  } 
  PID_ANGLE.OUT= PID_ANGLE.pout+ PID_ANGLE.dout;
}
void Get_Speed()                     //5ms执行一次
{  
  int qd1_result,qd2_result;
  qd1_result = Get_Count(CFTM0);
  qd2_result = Get_Count(CFTM1);
  
  Clean_Count(CFTM0);
  Clean_Count(CFTM1);

  //Distance+=(qd1_result+qd2_result)/5000.0;  //转化为距离  N/2/500 *0.2 = 
  CarSpeed=(qd1_result+qd2_result) * 0.006;//*250.0/6100.0;    //求出车速转换为M/S   N/2 /500 * L * T = N/2 /500 *0.2 * 250 = N /1000 * 50 = N * 0.05
   //if(CarSpeed>3.5)CarSpeed=3.5;
 
}
//速度控制量计算
void Speed_Control(void)
{         
   static float PreError[20]={0};
   float  SpeedError;
   uint8 i;
   SpeedError=Set_Speed-CarSpeed; 
 
   
  //求出最近20个偏差的总和作为积分项
   SpeedControlIntegral=0;
   for(i=0;i<19;i++)
   {
     PreError[i]=PreError[i+1]; 
     SpeedControlIntegral+=PreError[i];
   }
    PreError[19]=SpeedError;
    SpeedControlIntegral+=PreError[19];
   
  //速度更新
  SpeedControlOutOld=SpeedControlOutNew;
  SpeedControlOutNew = PID_SPEED.P*SpeedError + PID_SPEED.I*SpeedControlIntegral;   //PI控制
  //SpeedControlOutNew = SpeedControlOutOld*0.9 + SpeedControlOutNew*0.1;
}
//速度控制
void Speed_Control_Output(void) 
{ 
  float fValue; 
  fValue = SpeedControlOutNew - SpeedControlOutOld; 
  PID_SPEED.OUT = fValue * (SpeedCount+1)/50+SpeedControlOutOld;
  if( CarSpeed == 0 && SpeedControlOutNew > 0.5 )
    SpeedControlOutNew=0.5;
}

void roadturncal()  //转向控制程序  平躺
{
  AD_val_1 = adc_once(ADC_CHANNEL_AD14,ADC_12BIT);;  //左电感    
  AD_val_2 = adc_once(ADC_CHANNEL_AD4,ADC_12BIT);;  //右边电感 
  AD_val_3 = adc_once(ADC_CHANNEL_AD12,ADC_12BIT);;  // 中间电感    
  dis_AD_val_1=AD_val_1;
  dis_AD_val_2=AD_val_2;
  dis_AD_val_3=AD_val_3;
  //限幅
  if(AD_val_1>AD_val_1_max)		AD_val_1=AD_val_1_max;
  if(AD_val_2>AD_val_2_max)		AD_val_2=AD_val_2_max;
  //if(AD_val_3>AD_val_3_max)		AD_val_3=AD_val_3_max;
  
  if(AD_val_1<AD_val_1_min)		AD_val_1=AD_val_1_min;
  if(AD_val_2<AD_val_2_min)		AD_val_2=AD_val_2_min;			  
  if(AD_val_3<AD_val_3_min)		AD_val_3=AD_val_3_min;
  //if(AD_val_6<AD_val_6_min)		AD_val_6=AD_val_6_min;
  
  //归一化
  AD_val_1=100*(AD_val_1 - AD_val_1_min)/(AD_val_1_max-AD_val_1_min);
  AD_val_2=100*(AD_val_2 - AD_val_2_min)/(AD_val_2_max-AD_val_2_min);
  AD_val_3=100*(AD_val_3 - AD_val_3_min)/(AD_val_3_max-AD_val_3_min);
  disgy_AD_val_1 = AD_val_1;
  disgy_AD_val_2 = AD_val_2;
  disgy_AD_val_3 = AD_val_3;
  
  if (AD_val_1 == 0 && AD_val_2 == 0 && AD_val_3 == 0)
    Stop=1;
  if(AD_val_1 >35 && AD_val_2 >35 && AD_val_3 >35)
    Stop=0;
  if(Dir_last==2)  //原来在中间向左右偏离
  {
    if( (AD_val_2-AD_val_1)>20 || ((AD_val_2>AD_val_1) && AD_val_2<25)  || AD_val_1==0)  Dir_last = 1;   //上一状态为右转
    if( (AD_val_1-AD_val_2)>20 || ((AD_val_2<AD_val_1) && AD_val_1<25)  || AD_val_2==0)  Dir_last = 0;   //上一状态为左转
  }

  
  if(Dir_last==1)  //大弯或丢线
    Turn_Out=TURNPWM_MAX;
  if(Dir_last==0)
    Turn_Out = -TURNPWM_MAX;		//保持左转
  
  if(AD_val_1>=25 || AD_val_3>=40 || AD_val_2>=25 )
  {
      Dir_last = 2;
      DIR_ERROR=100*(AD_val_2 - AD_val_1)/(AD_val_2 + AD_val_1);          //偏差量
      //DIR_ERROR_STR=100*(AD_val_3 - 100)/(AD_val_3 + 100);          //偏差量
      DIR_ERROR=DIR_ERROR*(DIR_ERROR*DIR_ERROR/1250.0+2)/10;
      //DIR_ERROR_STR=DIR_ERROR_STR*(DIR_ERROR_STR*DIR_ERROR_STR/1250.0+2)/10;
      Push_And_Pull(DirectionErr,8,DIR_ERROR);
      
      if(Calculate_Length<8) 
        {  
          Calculate_Length++;
        }
      else
        {
          Error_Delta = -10*Slope_Calculate(0,Calculate_Length,DirectionErr);//求得斜率
         }
      
      DuoP=PID_TURN_Lie.P*ABS(DIR_ERROR);
      Set_Speed=SetSpeed;
      if(DuoP > 0.026)
        DuoP=0.026;
      Turn_Out =  DuoP * DIR_ERROR  + PID_TURN_Lie.D * Error_Delta + PID_TURN.D * Turn_Speed;  //电感的差 和 小车转动角速度融合
      if(Turn_Out >= TURNPWM_MAX) 
      {
        Turn_Out=TURNPWM_MAX;
        SetSpeed=SetSpeed_ZL;
      }
      if(Turn_Out <= -TURNPWM_MAX) 
      {
        Turn_Out=-TURNPWM_MAX;
        SetSpeed=SetSpeed_ZL;
      }
   }
  if(AD_val_3 >=150 && AD_val_2 <= 80 && AD_val_1<=80)
  {
    stringflag=1;
    stringcnt=0;
  }
  if(stringflag==1)
  {
    stringcnt++;
    if(stringcnt > 0)
    {
      BEEP_ON;
      SetSpeed=3.0;
      if(stringcnt > 10)
      {
        stringflag=0;
        BEEP_OFF;
      }
    }
  }
  Error_Delta_Deceleration=1.5;
  PID_TURN_Lie.OUT=Turn_Out;
}

//电机pwm值输出
void Moto_Out() 
{
 int L_Value,R_Value;
 //float Sum;

 
 if( ABS(PID_TURN_Lie.OUT) == TURNPWM_MAX ) //直立 + 速度 + 转向
 {  
   //BEEP_ON;
   if(PID_TURN_Lie.OUT > 0)
   {
     LeftMotorOut =  PID_ANGLE.OUT - PID_SPEED.OUT + PID_TURN_Lie.OUT;   //计算输出值
     RightMotorOut = PID_ANGLE.OUT - PID_SPEED.OUT - PID_TURN_Lie.OUT*Error_Delta_Deceleration; 
   }
   else
   {
     LeftMotorOut =  PID_ANGLE.OUT - PID_SPEED.OUT + PID_TURN_Lie.OUT*Error_Delta_Deceleration;   //计算输出值
     RightMotorOut = PID_ANGLE.OUT - PID_SPEED.OUT - PID_TURN_Lie.OUT; 
   }
  //BEEP_OFF; 
 }
 else
 {
   if(PID_TURN_Lie.OUT > 0)
   {
     LeftMotorOut  = PID_SPEED.OUT + PID_TURN_Lie.OUT+0.2;
     RightMotorOut = PID_SPEED.OUT - PID_TURN_Lie.OUT*Error_Delta_Deceleration+0.2;
   }
   else
   {
     LeftMotorOut  = PID_SPEED.OUT + PID_TURN_Lie.OUT*Error_Delta_Deceleration+0.2;
     RightMotorOut = PID_SPEED.OUT - PID_TURN_Lie.OUT+0.2;
   }
 }
  //LeftMotorOut=PID_SPEED.OUT;
  //RightMotorOut=PID_SPEED.OUT;
  if(RightMotorOut>0.99)RightMotorOut=0.99;                     
  if(RightMotorOut<-0.99)RightMotorOut=-0.99; 
  if(LeftMotorOut>0.99)LeftMotorOut=0.99;                     
  if(LeftMotorOut<-0.99)LeftMotorOut=-0.99; 
   
  
  L_Value=(int)(10000*LeftMotorOut);
  R_Value=(int)(10000*RightMotorOut);
 
  

  if(!Stop)
  {
    //dischargecnt++;
    //if(dischargecnt  > 5000)
      //dischargecnt=500;
    //电机1
    if(L_Value>=0) //正转
    {
       FTM_PWM_Duty(CFTM2,FTM_CH0,(int)(L_Value));//占空比精度为10000 
       FTM_PWM_Duty(CFTM2,FTM_CH1,0);
    }
    else   //反转
    {
       FTM_PWM_Duty(CFTM2,FTM_CH0,0);
       FTM_PWM_Duty(CFTM2,FTM_CH1,(int)(-L_Value));
    }
    //电机2
      if(R_Value>=0) //正转
    {
       FTM_PWM_Duty(CFTM2,FTM_CH2,(int)(R_Value)); 
       FTM_PWM_Duty(CFTM2,FTM_CH3,0);
    }
    else   //反转
    {
       FTM_PWM_Duty(CFTM2,FTM_CH2,0);
       FTM_PWM_Duty(CFTM2,FTM_CH3,(int)(-R_Value));
    }
  }
  else
  {
     //dischargecnt=0;
     FTM_PWM_Duty(CFTM2,FTM_CH0,0);
     FTM_PWM_Duty(CFTM2,FTM_CH1,0);
     FTM_PWM_Duty(CFTM2,FTM_CH2,0);
     FTM_PWM_Duty(CFTM2,FTM_CH3,0);
  }
}

float Slope_Calculate(uint8 begin,uint8 end,float *p)    //最小二乘法拟合斜率
{
  float xsum=0,ysum=0,xysum=0,x2sum=0;
   uint8 i=0;
   float result=0;
   static float resultlast;
   p=p+begin;
   for(i=begin;i<end;i++)
   {
	   xsum+=i;
	   ysum+=*p;
	   xysum+=i*(*p);
	   x2sum+=i*i;
	   p=p+1;
   }
  if((end-begin)*x2sum-xsum*xsum) //判断除数是否为零 
  {
    result=((end-begin)*xysum-xsum*ysum)/((end-begin)*x2sum-xsum*xsum);
    resultlast=result;
  }
  else
  {
   result=resultlast;
  }
  return result;
}

void Push_And_Pull(float *buff,int len,float newdata)
{
 int i;
 for(i=len-1;i>0;i--)
 {
   *(buff+i)=*(buff+i-1);
 }
   *buff=newdata; 
}

float  Turn_Out_Filter(float turn_out)    //转向控制输出滤波      
{
  float Turn_Out_Filtered; 
  static float Pre1_Error[4]; 
  Pre1_Error[3]=Pre1_Error[2];
  Pre1_Error[2]=Pre1_Error[1];
  Pre1_Error[1]=Pre1_Error[0];
  Pre1_Error[0]=turn_out;
  Turn_Out_Filtered=Pre1_Error[0]*0.4+Pre1_Error[1]*0.3+Pre1_Error[2]*0.2+Pre1_Error[3]*0.1;
  return Turn_Out_Filtered;
}
float  Middle_Err_Filter(float middle_err)    //中心偏差滤波      
{
  float Middle_Err_Fltered; 
  static float Pre3_Error[4]; 
  Pre3_Error[3]=Pre3_Error[2];
  Pre3_Error[2]=Pre3_Error[1];
  Pre3_Error[1]=Pre3_Error[0];
  Pre3_Error[0]=middle_err;
  Middle_Err_Fltered=Pre3_Error[0]*0.4+Pre3_Error[1]*0.3+Pre3_Error[2]*0.2+Pre3_Error[3]*0.1;
  return Middle_Err_Fltered;
}

void GET_MIN()
{  uint16 i;
   for(i=0;i<1200;i++)
   {
     AD_val_1 = adc_once(ADC_CHANNEL_AD13,ADC_12BIT);;  //左电感
     if(AD_val_1>=AD_val_1_max) 
       AD_val_1_max=AD_val_1;
     delayms(1);
     AD_val_2 = adc_once(ADC_CHANNEL_AD6,ADC_12BIT);;  //右电感
     if(AD_val_2>=AD_val_2_max) 
       AD_val_2_max=AD_val_2;
     delayms(1);
     AD_val_3 =  adc_once(ADC_CHANNEL_AD12,ADC_12BIT);; //中间电感	
     if(AD_val_3>=AD_val_3_max) 
       AD_val_3_max=AD_val_3;
     delayms(1);
   }
}

void GET_MAX()
{  uint16 i;
   uint32 sum=0;
   for(i=0;i<50;i++)   //Left-Min
   {
     AD_val_1 = adc_once(ADC_CHANNEL_AD14,ADC_12BIT);
     sum+=AD_val_1;
     delayms(5);
   }
   AD_val_1_min=sum/50;
   sum=0;
   for(i=0;i<50;i++)   //Right-Min
   {
     AD_val_2 = adc_once(ADC_CHANNEL_AD4,ADC_12BIT);
     sum+=AD_val_2;
     delayms(5);
   }
   AD_val_2_min=sum/50;
   sum=0;
   for(i=0;i<50;i++)   //Middle-Min
   {
     AD_val_3 = adc_once(ADC_CHANNEL_AD12,ADC_12BIT);	
     sum+=AD_val_3;
     delayms(5);
   }
   AD_val_3_min=sum/50;
   sum=0;
}