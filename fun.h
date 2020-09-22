#ifndef _fun_h
#define _fun_h

#include "headfile.h"


void  my_delay(long t);
void  delay(uint16_t ms);
int   mylabs(int  temp_data);
int16 myabs(int16 temp_data);
int   limit(int x, int y);
int   limit_ab(int x, int a, int b);
float Slope_Calculate(uint8 begin,uint8 end,float *p);   //最小二乘法拟合斜率

/*
 * 求最大值和最小值
 */
#define MAX( x, y ) ( ((x) > (y)) ? (x) : (y) )
#define MIN( x, y ) ( ((x) < (y)) ? (x) : (y) )


/*
 * 返回数组元素的个数
 */
#define ARR_SIZE( a ) ( sizeof( (a) ) / sizeof( ((a)[0]) ) )

/*
 * 宏定义实现返回绝对值（x里不能有自加自减的语句，否则变量出错）
 */
#define ABS(x) (((x) > 0) ? (x) : (-(x)))

/*
 * 获取结构体某成员偏移
 */
#define OFFSET(type, member)    (uint32)(&(((type *)0)->member))

/*
 * 确保x的范围为 min~max
 */
#define LIMIT(x,max,min)        (((x)<(min) ? (min) : ( (x)>(max) ? (max):(x) )))




/*
 * 取一个数据的各个位 
 */
#define BYTE0(Temp)       (*(char *)(&Temp))     
#define BYTE1(Temp)       (*((char *)(&Temp) + 1))
#define BYTE2(Temp)       (*((char *)(&Temp) + 2))
#define BYTE3(Temp)       (*((char *)(&Temp) + 3))














#endif 