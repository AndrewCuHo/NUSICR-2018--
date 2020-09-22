#include "headfile.h"


//int flag;
//uint8 workmode;  //С������״̬
//float Angle_OFF = 38;//34.5    ;//34.5 ;  // fֱ��ʱ��ʼ�Ƕ�ֵ
int ledcnt;
//extern int16  Gyro_X,Gyro_Y,Gyro_Z;
extern float SetSpeed_ZL;



////////////������///////////////////
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
     //workmode=0;  // 0����ֱ��   1:���ٶ�      2�� ֱ��+�ٶ�  3: ת��  4�� ֱ��+�ٶ�+ת��
     
     /////////////////////////////////////////////
     /////////////////////////////////////////////
     DisableInterrupts ;     
     GPIO_Init();
     FTM_Init();
     ADC_INIT();
     BEEP_ON;
     OLED_Init();                          //OLED      ��ʼ��
     delay(300);
     OLED_Fill(0);                     //��ʼ����
     BEEP_OFF;             
     I2C_Init();                       //��ʼ�����ٶ������Ǵ�����
     //  ���ٶȳ�ʼֵ
     delay(800);
     temp=0;
     for(mm=0;mm<20;mm++)
     {
       temp += Get_Y_Gyro(); 
       delay(2);
     }
     Gyro_Y_Offset = 0;//  temp/20;
     //  ���ٶȳ�ʼֵ
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
     PIT_Init(PIT_CHANNEL0,4*BUS_CLK_KHZ); //��ʱ�ж�   4ms ��ʼ�� 
     EnableInterrupts;
     PITDIENDINTERRUPT(PIT_CHANNEL0);
     while(1)  //С������
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
     Para_Init(); //������ʼ��
     while(1)
    {
      Check_BottonPress(); //��ⰴ��
      if(OLED_Refresh)  //��ʾ����
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

  // ҡ�˰���
  gpio_init(PTC2,GPI,1);   //��
  gpio_init(PTB4,GPI,1);   //��
  gpio_init(PTB5,GPI,1);   //��
  gpio_init(PTH0,GPI,1);     //���ļ�
  gpio_init(PTE6,GPI,1);   //��
  
  //3������
  gpio_init(PTD0,GPI,1);   //��
  gpio_init(PTD1,GPI,1);   //��
  //gpio_init(PTH6,GPI,1);   //��
  
  //3λ����
  gpio_init(PTA6,GPI,1);   //����
  gpio_init(PTA7,GPI,1);   //��
  gpio_init(PTF2,GPI,1);   //����
  
  // �ٶȷ���
  gpio_init(PTA0,GPI,1);   //����ת�������°��Ϊ A0
  gpio_init(PTG0,GPI,1);   //����ת������

  gpio_init (BEEP,GPO,1);   //������
}

///  С��ǰ�� CH0  CH2     С������  CH1  CH3
void FTM_Init()  //��������ʼ��
{    
     FTM_PWM_init(CFTM2, FTM_CH1, 10000, 0);     //PWM0 PTF1 �������
     FTM_PWM_init(CFTM2, FTM_CH0, 10000, 0);    //PWM1 PTF0  �������
     
     FTM_PWM_init(CFTM2, FTM_CH3, 10000, 0);     //PWM2 PTG5  �������
     FTM_PWM_init(CFTM2, FTM_CH2, 10000, 0);    //PWM3 PTG4  �������
     
     FTM_COUNT_INIT(CFTM0);                      ////��E0���������������м���
     FTM_COUNT_INIT(CFTM1);                      //��E7���������������м���
}

void ADC_INIT()
{
     ADC_Init(ADC_CHANNEL_AD4,ADC_12BIT);  //��
     ADC_Init(ADC_CHANNEL_AD6,ADC_12BIT);  //����
     ADC_Init(ADC_CHANNEL_AD7,ADC_12BIT);  //���ⰲ��
     ADC_Init(ADC_CHANNEL_AD12,ADC_12BIT); //��
     ADC_Init(ADC_CHANNEL_AD13,ADC_12BIT); //����
     ADC_Init(ADC_CHANNEL_AD14,ADC_12BIT); //��
     ADC_Init(ADC_CHANNEL_AD15,ADC_12BIT); //F7  //��ص�ѹ�ɼ�ʵ��  //C2
}



//��ʱ��0�жϺ���
void PIT0_ISR(void)
{  uint16   AD_BAT;
   static uint8 flag_100ms,cnt;
     RunTime=RunTime+0.002;  //������ʱ
  flag_100ms++;
  ledcnt++;
  if(ledcnt==250)  //1S
  {  gpio_turn(PTB1); 
     gpio_turn(PTD2); 
     gpio_turn(PTE0); 
     gpio_turn(PTE5); 
     AD_BAT = adc_once(ADC_CHANNEL_AD15,ADC_12BIT);;  // �м��� 
     Voltage = AD_BAT * Vol_ratio;
     ledcnt=0;
  }
  if(flag_100ms>50)
  {
   flag_100ms=0;  
   Speed_Control();  //100ms����һ���ٶȿ���
   SpeedCount=0;
  }
  cnt++;
   if(cnt==1)      //4ms����һ��
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
 
   PIT->CHANNEL[0].TFLG |= PIT_TFLG_TIF_MASK;//����жϱ�־λ  
  
}



