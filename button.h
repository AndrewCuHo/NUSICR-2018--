/*!
 * @file       button.h
 * @brief      按键相关头文件
 */

#ifndef _BUTTON_H_
#define _BUTTON_H_

#define BT_CANCEL    PTD0 //BT3
#define BT_RIGHT     PTB5 //BT6
#define BT_UP        PTB4//BT2
#define BT_DOWN      PTE6 //BT5
#define BT_YES       PTH0  //BT1
#define BT_LEFT      PTC2 //BT4

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

#define BT_SHOW_IN   gpio_get(BT_CANCEL)
#define BT_LEFT_IN   gpio_get(BT_LEFT)
#define BT_UP_IN     gpio_get(BT_UP)
#define BT_DOWN_IN   gpio_get(BT_DOWN)
#define BT_YES_IN    gpio_get(BT_YES)
#define BT_RIGHT_IN  gpio_get(BT_RIGHT)

void button_init();
void switch_init();

#endif /* _BUTTON_H_ */