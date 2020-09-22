/*!
 * @file       LED.C
 * @brief      LED常用函数

 */
#include "headfile.h"
 /*
    函数作用 :板载led初始化
    注：核心板上有三个可编程LED，端口分别是 PTA19,PTE6,PTC18,低电平点亮
      端口定义在 led.h
 */  

void button_init()
{
  gpio_init (BT_CANCEL, GPI,1);  
  gpio_init (BT_LEFT, GPI,1);
  gpio_init (BT_UP, GPI,1);
  gpio_init (BT_DOWN, GPI,1);
  gpio_init (BT_YES, GPI,1);
  gpio_init (BT_RIGHT, GPI,1);  
  gpio_init (BEEP,GPO,0);
}

void switch_init()
{
  gpio_init (SW1, GPI,1);
  gpio_init (SW2, GPI,1);
  gpio_init (SW3, GPI,1); 
//  gpio_init (SW4, GPI,1);
}


