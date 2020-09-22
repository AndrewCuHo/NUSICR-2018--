#ifndef __DEBUG_H__
#define __DEBUG_H__
#define Vol_ratio 5.0/4096*3.15
#define ABS(x) (((x) > 0) ? (x) : (-(x)))   //（x里不能有自加自减的语句，否则变量出错）

#define BT_CANCEL    PTD0 //BT3
#define BT_RIGHT     PTB5 //BT6
#define BT_UP        PTB4//BT2
#define BT_DOWN      PTE6 //BT5
#define BT_YES       PTH0  //BT1
#define BT_LEFT      PTC2 //BT4
#define BT_CarReset  PTD1
#define BT_CarStar   PTH6
#define SW1      PTA6
#define SW2      PTA7
#define SW3      PTF2

//#define SW4      PTE1

#define SW1_IN   gpio_get(SW1)
#define SW2_IN   gpio_get(SW2)
#define SW3_IN   gpio_get(SW3)
//#define SW4_IN   gpio_get(SW4)

#define BEEP      PTH7
#define BEEP_ON   gpio_set (BEEP,0)   //0：发声 1：静默
#define BEEP_OFF  gpio_set (BEEP,1)

#define BT_SHOW_IN      gpio_get(BT_CANCEL)
#define BT_LEFT_IN      gpio_get(BT_LEFT)
#define BT_UP_IN        gpio_get(BT_UP)
#define BT_DOWN_IN      gpio_get(BT_DOWN)
#define BT_YES_IN       gpio_get(BT_YES)
#define BT_RIGHT_IN     gpio_get(BT_RIGHT)
#define BT_CarReset_IN  gpio_get(BT_CarReset)  
#define BT_CarStar_IN   gpio_get(BT_CarStar)
extern float Variable[20];
extern float sVariable[20];
extern float Parameter[20];
extern float Control_Para[15];
extern float RunTime;
extern float Voltage;
extern uint8 Starting;
extern int  Start_Cnt;
extern  uint8 Page_Index,Para_Index,Para_Checked,OLED_Refresh,Camera_Drawed;
extern uint8 send_data,Uart_Send;

extern  uint8 Stop;
void Check_BottonPress();
void Read_Switch();
void Para_Update();
void OLED_Draw_UI();  //画出界面
void delay(uint16_t ms);
#endif 