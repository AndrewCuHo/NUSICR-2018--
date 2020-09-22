/*!
 * @file       LED.C
 * @brief      LED���ú���

 */
#include "headfile.h"
 /*
    �������� :����led��ʼ��
    ע�����İ����������ɱ��LED���˿ڷֱ��� PTA19,PTE6,PTC18,�͵�ƽ����
      �˿ڶ����� led.h
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


