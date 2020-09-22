#include "headfile.h"
float sVariable[20]; //传感器变量
float Control_Para[15];
float Voltage;
float RunTime;
int   Start_Cnt=0;
uint8 Para_Index_Limit=7;       //一页最多有7个变量序号
uint8 Page_Index=3,Para_Index=1,Light_Tower_Index=0,Para_Checked=0,OLED_Refresh=0;
float Step[6]={0.0001,0.001,0.01,0.1,1.0,10.0};   //默认调节步长为0.01
unsigned char Step_Index=2;
//对应不同的页面
char Para_Name[7][12]={"Set_SPEED\0","PID_ANGLE.P\0","PID_ANGLE.D\0","PID_SPEED.P\0",
"PID_SPEED.I\0","PID_DIREC.P\0","PID_DIREC.D\0"};
//char Debug_Mode[4][12]={"Normal  \0","UpRight\0","NoSpeed\0","NoDirec\0"};  //调试模式 正常 直立 没有速度 ，没有方向

char Para_Name1[7][12]={"SetSpeed_ZL\0","Set_Angle\0","Acc_Offset\0","Fuzzy_kp",
"Fuzzy_kd","STurnSpeed\0","BTurnAngle\0"};

extern int16 disangle;
extern int16  Gyro_X,Gyro_Y,Gyro_Z;
extern uint16 dis_AD_val_1,dis_AD_val_2,dis_AD_val_3;
extern float Turn_Out,DIR_ERROR,CarSpeed,Error_Delta,Turn_Speed,RightMotorOut,LeftMotorOut,SetSpeed_ZL;
extern PID PID_SPEED;
extern uint16   disgy_AD_val_1,disgy_AD_val_2,disgy_AD_val_3;
void Para_Update()
{ 
  SetSpeed=Control_Para[0];
  PID_ANGLE.P=Control_Para[1];
  PID_ANGLE.D=Control_Para[2];
  PID_SPEED.P=Control_Para[3];
  PID_SPEED.I=Control_Para[4];
  PID_TURN_Lie.P=Control_Para[5];
  PID_TURN_Lie.D=Control_Para[6]; 
  SetSpeed_ZL=Control_Para[7];
}


void OLED_Draw_UI()  //画出界面
{ 
   uint8 i;
   if(Page_Index<=1) 
   {
     OLED_P6x8Str(0,0,"Voltage=");                          //显示电池电压
     OLED_PrintValueF(48, 0,Voltage,2);                     
     OLED_PrintValueF(72, 0,Step[Step_Index],5);            //显示调节步进值
     if(Para_Index==7)
     {
        reverse=1; 
         OLED_P6x8Str(116,0,"EE"); 
        reverse=0;
     }
     else  
     {
        OLED_P6x8Str(116,0,"EE"); 
     }
     OLED_Set_Pos(122,7);
     OLED_P6x8Char(Page_Index+48);                         //写出页面序号
   }
  /////////////////////////////////////////////////////////第1页  PID调节
  if(Page_Index==0)                
  {
    for(i=0;i<7;i++)
    {
      if(i==Para_Index&&Para_Checked==false)
      {
       reverse=1;
       OLED_P6x8Str(0,i+1,Para_Name[i]);   //将参量名反转显示
       reverse=0;
      }
      else OLED_P6x8Str(0,i+1,Para_Name[i]);

      
      
      if(i==Para_Index&&Para_Checked)
      {
        reverse=1;
        OLED_PrintValueF(72, i+1,Control_Para[i],5);
        reverse=0;
      }
      else  OLED_PrintValueF(72, i+1,Control_Para[i],5);
      
      OLED_Set_Pos(116,i+1);

    }
  }
  /////////////////////////////////////////////////////////第2页   参数调节
    else if(Page_Index==1)
  {  
     for(i=0;i<7;i++)
    {
     if(i==Para_Index&&Para_Checked==false)
      {
       reverse=1;
       OLED_P6x8Str(0,i+1,Para_Name1[i]);   //将参量名反转显示
       reverse=0;
      }
      else OLED_P6x8Str(0,i+1,Para_Name1[i]);  
     
      if(i==Para_Index&&Para_Checked)
      {
        reverse=1;
        OLED_PrintValueF(72, i+1,Control_Para[i+7],5);
        reverse=0;
      }
      else  OLED_PrintValueF(72, i+1,Control_Para[i+7],5);    
    
      OLED_Set_Pos(116,i+1);
     }
    }
    /////////////////////////////////////////////////////////第3页 状态显示
  else if(Page_Index==2)
  {
    OLED_P6x8Str(0,0,"L_Value");
    OLED_PrintValueF(72, 0,LeftMotorOut,4);
    OLED_P6x8Str(0,1,"R_Value");
    OLED_PrintValueF(72, 1,RightMotorOut,4);
    OLED_P6x8Str(0,2,"Turn_Out");
    OLED_PrintValueF(72, 2,Turn_Out,4);
    OLED_P6x8Str(0,3,"RunTime");
    OLED_PrintValueF(72, 3,RunTime,4);
    OLED_P6x8Str(0,4,"CarSpeed");
    OLED_PrintValueF(72, 4,CarSpeed,4);   
    OLED_P6x8Str(0,5,"PID_SPEED:");
    OLED_P6x8Str(0,6,"Error_Delta:");
    OLED_PrintValueF(72, 5,PID_SPEED.OUT,4);   
    OLED_PrintValueF(72, 6,Error_Delta,4);
    OLED_PrintValueI(2,7,disangle);
    OLED_PrintValueI(72, 7,DIR_ERROR);   
    reverse=0;
     
  } 
  ////////////////////////////////////////////////////////////第4页 传感器显示
  else if(Page_Index==3) 
  {
    OLED_P6x8Str(0,0,"AD_1=");
    OLED_PrintValueI(36,0,dis_AD_val_1);
    OLED_PrintValueI(92,0,disgy_AD_val_1);
    OLED_P6x8Str(0,1,"AD_3=");
    OLED_PrintValueI(36,1,dis_AD_val_3);
    OLED_PrintValueI(92,1,disgy_AD_val_3);
    OLED_P6x8Str(0,2,"AD_2=");
    OLED_PrintValueI(36, 2,dis_AD_val_2);
    OLED_PrintValueI(92,2,disgy_AD_val_2);
    
    OLED_P6x8Str(0,4,"Gyro_X_off");
    OLED_PrintValueI(72,4,Gyro_X_Offset);
    OLED_P6x8Str(0,5,"Gyro_Y_off");
    OLED_PrintValueI(72,5,Gyro_Y_Offset);
    
    OLED_P6x8Str(0,6,"Gyro_X");
    OLED_PrintValueI(72,6,Gyro_X);
    OLED_P6x8Str(0,7,"Gyro_Y");
    OLED_PrintValueI(72,7,Gyro_Y);
    
  }
  
}


/*
 * 读拨码开关的值
 */
void Read_Switch() 
{
  if(gpio_get(SW1)==0)  //拨码开关1功能
  {
   Uart_Send=true;
  }
  else           
  {
    Uart_Send=false;
  }

  if(gpio_get(SW3)==0)   //拨码开关3功能
  {
       
  } 
  else 
  {
    
  }

} 

/*
 * 检测按键是否按下  
 */
void Check_BottonPress()
{
  
      //显示按键  用于消除屏幕乱码的影响
     if(BT_SHOW_IN==0)
   {
      //去抖
       BEEP_ON;
       delay(30);
       BEEP_OFF;
      if(BT_SHOW_IN==0)
      {    
          if(OLED_Refresh==false)
        {
         OLED_Init();
         OLED_Refresh=true;
        }
        else
        {
          OLED_Refresh=false;
          OLED_Fill(0x00);       
        }
      }
      while(BT_SHOW_IN==0);  //直到按键松开再运行
      delay(30);
   } 
   
   //按键1 yes
   if(BT_YES_IN==0) 
   {
     //去抖
       BEEP_ON;
       delay(30);
       BEEP_OFF;
      if(BT_YES_IN==0&&OLED_Refresh)
      {    
       if(Para_Index==7)   //7是页码显示栏  不可调
       { 
         //EEPROM_Save();
         Para_Index=0; 
       }
       else
       {
 
         if(Para_Checked==false&&((Page_Index==1)||(Page_Index==0))) Para_Checked=true;
         else Para_Checked=false;       

       }
      }
      while(BT_YES_IN==0); //直到按键松开再运行
   }
   //按键2 Left_L
   if(BT_LEFT_IN==0&&OLED_Refresh)
   {
      //去抖
       BEEP_ON;
       delay(30);
       BEEP_OFF;
      if(BT_LEFT_IN==0)
      {
        if(Para_Checked) //检测是否是处于可调节状态
        {
          if(Step_Index==5) 
          Step_Index=5;   //最大的步长为10
          else Step_Index++;
        }
        else 
        { 
          Para_Index=0;  //保证了在翻页翻到调节页面时可以从第一个开始调节
          if(Page_Index==0) 
            Page_Index=3; //当参数没被选中的时候，按左右键翻页
          else 
            Page_Index--;
          OLED_Fill(0);//清屏 
        }
      }
      while(BT_LEFT_IN==0);//直到按键松开再运行
   } 
   //按键6 Right_L
   if(BT_RIGHT_IN==0&&OLED_Refresh)
   {
      //去抖
       BEEP_ON;
       delay(30);
       BEEP_OFF;
      if(BT_RIGHT_IN==0)
      {
        if(Para_Checked) 
        {
          if(Step_Index==0) 
           Step_Index=0;//最小的步长为0.0001
          else
          {
            Step_Index--;
          }
        }
        else 
        { 
          Para_Index=0;
          if(Page_Index==3) Page_Index=0;
          else Page_Index++;
         OLED_Fill(0);//清屏 
        }
      }
      while(BT_RIGHT_IN==0);      //直到按键松开再运行
   }
   //按键3 up
   if(BT_UP_IN==0&&OLED_Refresh)
   {
       BEEP_ON;
       delay(30);
       BEEP_OFF;
      if(BT_UP_IN==0)
      {
   
          if(Para_Checked==false)
          {
           if(Para_Index==0) Para_Index=Para_Index_Limit;
           else Para_Index-=1;
          }
          else
          {
              if(Page_Index==0&&Para_Index<=6)                    //修改第0页的参数
            {
              Control_Para[Para_Index]+=Step[Step_Index];
            }
            
            if(Page_Index==1&&Para_Index<=6)                    //修改第1页的参数
            {
              Control_Para[Para_Index+7]+=Step[Step_Index];
            } 
            Para_Update();
          }
      }  
      while(BT_UP_IN==0);//直到按键松开再运行  
   }
   //按键4 down
   if(BT_DOWN_IN==0&&OLED_Refresh)
   {
       BEEP_ON;
       delay(30);
       BEEP_OFF;
      if(BT_DOWN_IN==0)
      {
          if(Para_Checked==false)
          {             
            if(Para_Index==Para_Index_Limit)Para_Index=0;   //防止序号超出范围
            else  Para_Index+=1; 
          }
           else 
           {
              if(Page_Index==0&&Para_Index<=6)                    //修改第0页的参数
            {
              Control_Para[Para_Index]-=Step[Step_Index];
            }
             
             if(Page_Index==1&&Para_Index<=6)                    //修改第1页的参数
            {
              Control_Para[Para_Index+7]-=Step[Step_Index];
            }
            Para_Update();
          }
      }

      while(BT_DOWN_IN==0);  //直到按键松开再运行
   }
   if(BT_CarReset_IN == 0)
   {
       BEEP_ON;
       delay(30);
       BEEP_OFF;
       if(BT_CarReset_IN == 0)
       {
         Stop=0;
       }
       while(BT_CarReset_IN == 0);
   }
}

void delay(uint16_t ms)
{
  uint16_t ii,jj;
  if (ms<1) ms=1;
  ms = ms*12 ;
  for(ii=0;ii<ms;ii++)
    for(jj=0;jj<1335;jj++);  //16MHz--1ms
  //   for(jj=0;jj<4006;jj++);  //48MHz--1ms
  //for(jj=0;jj<5341;jj++);    //64MHz--1ms
}


