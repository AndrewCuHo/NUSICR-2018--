#include "isr.h"





int stime = 0;  //系统时间





//定时器1中断函数
void PIT1_ISR(void)
{
    PIT->CHANNEL[1].TFLG |= PIT_TFLG_TIF_MASK;//清楚中断标志位
  

}






//KBI0中断函数
void KBI0_Isr(void)	
{
  
     KBI0->SC |= KBI_SC_KBACK_MASK;       /* clear interrupt flag */
     KBI0->SC |= KBI_SC_RSTKBSP_MASK ;    //清除中断标志位

  

}






//KBI1中断函数
void KBI1_Isr(void)	
{
   
     KBI1->SC |= KBI_SC_KBACK_MASK;                /* clear interrupt flag */
     KBI1->SC |= KBI_SC_RSTKBSP_MASK ;             //清除中断标志位
}

  //定时器0中断函数
void UART0_ISR(void)
{ 
  
}
  