#include "headfile.h"


//int flag;
//uint8 workmode;  //小车工作状态
//float Angle_OFF = 38;//34.5    ;//34.5 ;  // f直立时初始角度值
int ledcnt;
//extern int16  Gyro_X,Gyro_Y,Gyro_Z;
extern float SetSpeed_ZL;



////////////函数区///////////////////
void GPIO_Init();
void FTM_Init();
void ADC_INIT();

void Para_Init()
{
 PID_ANGLE.P=0.09;
 PID_ANGLE.D=0.0035;
 PID_SPEED.P=0.17;
 PID_SPEED.I=0.0015;
 PID_TURN.P=0.075;//0.02;
 PID_TURN_Lie.P=0.0029;
 PID_TURN_Lie.D=0.025;
 PID_TURN.D=0.0006;//0.005; 
 //Fuzzy_Kp=0.005;
 //Fuzzy_Kd=0.0005;
 SetSpeed= 2.4;
 SetSpeed_ZL=1.7;
 Acc_Offset=0;//-3700;
 
 Control_Para[0]= SetSpeed;
 Control_Para[1]= PID_ANGLE.P;
 Control_Para[2]= PID_ANGLE.D;
 Control_Para[3]= PID_SPEED.P;
 Control_Para[4]= PID_SPEED.I;
 Control_Para[5]= PID_TURN_Lie.P;
 Control_Para[6]= PID_TURN_Lie.D;
 Control_Para[7]= SetSpeed_ZL;
}

void main(void)
{    int temp;
     unsigned char mm;
     //workmode=0;  // 0：纯直立   1:纯速度      2： 直立+速度  3: 转向  4： 直立+速度+转向
     
     /////////////////////////////////////////////
     /////////////////////////////////////////////
     DisableInterrupts ;     
     GPIO_Init();
     FTM_Init();
     ADC_INIT();
     BEEP_ON;
     OLED_Init();                          //OLED      初始化
     delay(300);
     OLED_Fill(0);                     //初始清屏
     BEEP_OFF;             
     I2C_Init();                       //初始化加速度陀螺仪传感器
     //  角速度初始值
     delay(800);
     temp=0;
     for(mm=0;mm<20;mm++)
     {
       temp += Get_Y_Gyro(); 
       delay(2);
     }
     Gyro_Y_Offset = 0;//  temp/20;
     //  角速度初始值
     temp=0;
     for(mm=0;mm<20;mm++)
     {
       temp += Get_X_Gyro();
       delay(2);
     }
     Gyro_X_Offset = 0;// temp/20;
     OLED_Refresh=true;
     OLED_P6x8Str(0,3,"first open ");
     OLED_P6x8Str(0,4,"Electric machinery");  
     OLED_P6x8Str(0,5,"then Press D0");
     //OLED_PrintValueI(80,1,Gyro_Y_Offset);
     ///////////////////////////////////
     PIT_Init(PIT_CHANNEL0,4*BUS_CLK_KHZ); //定时中断   4ms 初始化 
     EnableInterrupts;
     PITDIENDINTERRUPT(PIT_CHANNEL0);
     while(1)  //小车启动
     {
       if(gpio_get(BT_CANCEL) == 0)
       {
         BEEP_ON;
         delay(30);
         BEEP_OFF;
         while(gpio_get(BT_CANCEL) == 0);
         delay(100);
         break;
       }
     }
     PITENDINTERRUPT(PIT_CHANNEL0);
     Para_Init(); //参数初始化
     while(1)
    {
      Check_BottonPress(); //检测按键
      if(OLED_Refresh)  //显示画面
      {
        OLED_Draw_UI();
      }      
    }
}

void GPIO_Init()
{ 
  SIM->SOPT0 = SIM->SOPT0 & 0xfffd;   
  //LED
  gpio_init(PTB1,GPO,0);   
  gpio_init(PTD2,GPO,0); 
  gpio_init(PTC5,GPO,0); 
  gpio_init(PTE5,GPO,0); 

  // 摇杆按键
  gpio_init(PTC2,GPI,1);   //左
  gpio_init(PTB4,GPI,1);   //上
  gpio_init(PTB5,GPI,1);   //右
  gpio_init(PTH0,GPI,1);     //中心键
  gpio_init(PTE6,GPI,1);   //下
  
  //3个按键
  gpio_init(PTD0,GPI,1);   //下
  gpio_init(PTD1,GPI,1);   //中
  //gpio_init(PTH6,GPI,1);   //上
  
  //3位拔码
  gpio_init(PTA6,GPI,1);   //左上
  gpio_init(PTA7,GPI,1);   //上
  gpio_init(PTF2,GPI,1);   //右上
  
  // 速度方向
  gpio_init(PTA0,GPI,1);   //车轮转动方向，新板改为 A0
  gpio_init(PTG0,GPI,1);   //车轮转动方向

  gpio_init (BEEP,GPO,1);   //蜂鸣器
}

///  小车前进 CH0  CH2     小车后退  CH1  CH3
void FTM_Init()  //编码器初始化
{    
     FTM_PWM_init(CFTM2, FTM_CH1, 10000, 0);     //PWM0 PTF1 电机驱动
     FTM_PWM_init(CFTM2, FTM_CH0, 10000, 0);    //PWM1 PTF0  电机驱动
     
     FTM_PWM_init(CFTM2, FTM_CH3, 10000, 0);     //PWM2 PTG5  电机驱动
     FTM_PWM_init(CFTM2, FTM_CH2, 10000, 0);    //PWM3 PTG4  电机驱动
     
     FTM_COUNT_INIT(CFTM0);                      ////对E0引脚输入的脉冲进行计数
     FTM_COUNT_INIT(CFTM1);                      //对E7引脚输入的脉冲进行计数
}

void ADC_INIT()
{
     ADC_Init(ADC_CHANNEL_AD4,ADC_12BIT);  //右
     ADC_Init(ADC_CHANNEL_AD6,ADC_12BIT);  //右上
     ADC_Init(ADC_CHANNEL_AD7,ADC_12BIT);  //随意安排
     ADC_Init(ADC_CHANNEL_AD12,ADC_12BIT); //中
     ADC_Init(ADC_CHANNEL_AD13,ADC_12BIT); //左上
     ADC_Init(ADC_CHANNEL_AD14,ADC_12BIT); //左
     ADC_Init(ADC_CHANNEL_AD15,ADC_12BIT); //F7  //电池电压采集实验  //C2
}



//定时器0中断函数
void PIT0_ISR(void)
{  uint16   AD_BAT;
   static uint8 flag_100ms,cnt;
     RunTime=RunTime+0.002;  //用作计时
  flag_100ms++;
  ledcnt++;
  if(ledcnt==250)  //1S
  {  gpio_turn(PTB1); 
     gpio_turn(PTD2); 
     gpio_turn(PTE0); 
     gpio_turn(PTE5); 
     AD_BAT = adc_once(ADC_CHANNEL_AD15,ADC_12BIT);;  // 中间电感 
     Voltage = AD_BAT * Vol_ratio;
     ledcnt=0;
  }
  if(flag_100ms>50)
  {
   flag_100ms=0;  
   Speed_Control();  //100ms进行一次速度控制
   SpeedCount=0;
  }
  cnt++;
   if(cnt==1)      //4ms运行一次
   { 
     Get_Attitude(); 
     Angle_Calculate();
     Angle_Control(); 
     Get_Speed();
     roadturncal();
   }
   if(cnt>=2)
   {
     cnt=0;
   }
     SpeedCount++;
     Speed_Control_Output();
     Moto_Out();
 
   PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;//清楚中断标志位  
  
}



